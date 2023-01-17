#include <nuttx/config.h>

// #include <sdk/config.h>
#include <arch/board/board.h>
#include <arch/board/cxd56_sdcard.h>
#include <arch/chip/gnss.h>
#include <arch/chip/pin.h>
#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <nuttx/arch.h>
#include <nuttx/config.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/boardctl.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#ifdef CONFIG_CXD56_ADC
#include <arch/chip/adc.h>
#include <arch/chip/scu.h>
#endif

#ifndef CONFIG_EXAMPLES_ADC_MONITOR_DEVPATH
#define CONFIG_EXAMPLES_ADC_MONITOR_DEVPATH "/dev/hpadc0"
#endif

/* GNSS */

#define GNSS_POLL_FD_NUM 1
#define GNSS_POLL_TIMEOUT_FOREVER -1
#define MY_GNSS_SIG 18

struct cxd56_gnss_dms_s {
    int8_t sign;
    uint8_t degree;
    uint8_t minute;
    uint32_t frac;
};

static uint32_t posfixflag;
static struct cxd56_gnss_positiondata_s posdat;

static void double_to_dmf(double x, struct cxd56_gnss_dms_s *dmf) {
    int b;
    int d;
    int m;
    double f;
    double t;

    if(x < 0) {
        b = 1;
        x = -x;
    } else {
        b = 0;
    }
    d = (int)x; /* = floor(x), x is always positive */
    t = (x - d) * 60;
    m = (int)t; /* = floor(t), t is always positive */
    f = (t - m) * 10000;

    dmf->sign = b;
    dmf->degree = d;
    dmf->minute = m;
    dmf->frac = f;
}

static int read_and_print(int fd) {
    int ret;
    struct cxd56_gnss_dms_s dmf;

    /* Read POS data. */

    ret = read(fd, &posdat, sizeof(posdat));
    if(ret < 0) {
        printf("read error\n");
        goto _err;
    } else if(ret != sizeof(posdat)) {
        ret = ERROR;
        printf("read size error\n");
        goto _err;
    } else {
        ret = OK;
    }

    /* Print POS data. */

    /* Print time. */

    printf(">Hour:%d, minute:%d, sec:%d, usec:%ld\n", posdat.receiver.time.hour, posdat.receiver.time.minute, posdat.receiver.time.sec, posdat.receiver.time.usec);
    if(posdat.receiver.pos_fixmode != CXD56_GNSS_PVT_POSFIX_INVALID) {
        /* 2D fix or 3D fix.
         * Convert latitude and longitude into dmf format and print it. */

        posfixflag = 1;

        double_to_dmf(posdat.receiver.latitude, &dmf);
        printf(">LAT %d.%d.%04ld\n", dmf.degree, dmf.minute, dmf.frac);

        double_to_dmf(posdat.receiver.longitude, &dmf);
        printf(">LNG %d.%d.%04ld\n", dmf.degree, dmf.minute, dmf.frac);
    } else {
        /* No measurement. */

        printf(">No Positioning Data\n");
    }

_err:
    return ret;
}

static int gnss_setparams(int fd) {
    int ret = 0;
    uint32_t set_satellite;
    struct cxd56_gnss_ope_mode_param_s set_opemode;

    /* Set the GNSS operation interval. */

    set_opemode.mode = 1;     /* Operation mode:Normal(default). */
    set_opemode.cycle = 1000; /* Position notify cycle(msec step). */

    ret = ioctl(fd, CXD56_GNSS_IOCTL_SET_OPE_MODE, (uint32_t)&set_opemode);
    if(ret < 0) {
        printf("ioctl(CXD56_GNSS_IOCTL_SET_OPE_MODE) NG!!\n");
        goto _err;
    }

    /* Set the type of satellite system used by GNSS. */

    set_satellite = CXD56_GNSS_SAT_GPS | CXD56_GNSS_SAT_GLONASS | CXD56_GNSS_SAT_SBAS | CXD56_GNSS_SAT_QZ_L1CA | CXD56_GNSS_SAT_QZ_L1S;

    ret = ioctl(fd, CXD56_GNSS_IOCTL_SELECT_SATELLITE_SYSTEM, set_satellite);
    if(ret < 0) {
        printf("ioctl(CXD56_GNSS_IOCTL_SELECT_SATELLITE_SYSTEM) NG!!\n");
        goto _err;
    }

_err:
    return ret;
}

#define BUFSIZE 2048
char buf[BUFSIZE];
int ret;
int fd;
int fd_gnss;

int16_t logs[BUFSIZE];

double voltage(int value) {
    double val = (double)value;
    return (1.39668 * (val + 32768.0) / 65536.0 + 0.00165818) * (27.0 + 10.0) / 10.0;
}

/* CSV */

#define CSVFILE_ROOTPATH "/mnt/sd0"
#define MAX_PATH_LENGTH 128
static char csvfname[MAX_PATH_LENGTH];
static FILE *fd_csv;

/* RTC */
#define TIMEZONE_JST (60 * 60 * 9)
long time_diff;

/* MAIN */

int main(int argc, FAR char *argv[]) {
    struct timespec ts;
    struct tm tm;
    struct timespec tszero;
    tszero.tv_sec = 0;
    tszero.tv_nsec = 0;
    struct cxd56_gnss_dms_s dmf;

    sigset_t mask;
    struct cxd56_gnss_signal_setting_s setting;

    int errval = 0;
    char *buftop = &buf[0];

    fd = open(CONFIG_EXAMPLES_ADC_MONITOR_DEVPATH, O_RDONLY);
    if(fd < 0) {
        printf("open %s failed: %d\n", CONFIG_EXAMPLES_ADC_MONITOR_DEVPATH, errno);
        errval = 4;
        goto errout;
    }

    ret = ioctl(fd, SCUIOC_SETFIFOMODE, 1);
    if(ret < 0) {
        errval = errno;
        printf("ioctl(SETFIFOMODE) failed: %d\n", errval);
        goto errout_with_dev;
    }

    ret = ioctl(fd, ANIOC_CXD56_START, 0);
    if(ret < 0) {
        errval = errno;
        printf("ioctl(START) failed: %d\n", errval);
        goto errout_with_dev;
    }

    // gnss

    fd_gnss = open("/dev/gps", O_RDONLY);
    if(fd_gnss < 0) {
        printf("open error:%d,%d\n", fd_gnss, errno);
        return -ENODEV;
    }

    sigemptyset(&mask);
    sigaddset(&mask, MY_GNSS_SIG);
    ret = sigprocmask(SIG_BLOCK, &mask, NULL);
    if(ret != OK) {
        printf("sigprocmask failed. %d\n", ret);
        goto _err;
    }

    setting.fd = fd_gnss;
    setting.enable = 1;
    setting.gnsssig = CXD56_GNSS_SIG_GNSS;
    setting.signo = MY_GNSS_SIG;
    setting.data = NULL;

    ret = ioctl(fd_gnss, CXD56_GNSS_IOCTL_SIGNAL_SET, (unsigned long)&setting);
    if(ret < 0) {
        printf("signal error\n");
        goto _err;
    }

    ret = gnss_setparams(fd_gnss);
    if(ret != OK) {
        printf("gnss_setparams failed. %d\n", ret);
        // goto _err;
    }

    // posperiod = 200;
    posfixflag = 0;

    ret = ioctl(fd_gnss, CXD56_GNSS_IOCTL_START, CXD56_GNSS_STMOD_HOT);
    if(ret < 0) {
        printf("start GNSS ERROR %d\n", errno);
        // goto _err;
    } else {
        printf("start GNSS OK\n");
    }

    // csv
    board_sdcard_initialize();
    snprintf(csvfname, MAX_PATH_LENGTH, "%s/%s", CSVFILE_ROOTPATH, "log.csv");

    int counter = 0;
    int kiroku = 0;
    int diff = 0;
    bool flag = false;
    int16_t sample;
    int plot_center = 40;

    int temp = 0;

    // rtc

    ret = sigwaitinfo(&mask, NULL);
    printf("sigtimedwait %d, %d\n", ret, errno);
    ret = read(fd_gnss, &posdat, sizeof(posdat));
    printf(">%d, %d/%d/%d Hour:%d, minute:%d, sec:%d, usec:%ld\n", temp - 20000, posdat.receiver.date.year, posdat.receiver.date.month, posdat.receiver.date.day, posdat.receiver.time.hour, posdat.receiver.time.minute, posdat.receiver.time.sec, posdat.receiver.time.usec);

    tm.tm_year = posdat.receiver.date.year - 1900;
    tm.tm_mon = posdat.receiver.date.month - 1;
    tm.tm_mday = posdat.receiver.date.day;
    tm.tm_hour = posdat.receiver.time.hour;
    tm.tm_min = posdat.receiver.time.minute;
    tm.tm_sec = posdat.receiver.time.sec;
    ts.tv_nsec = (int)(posdat.receiver.time.usec) * 1000;
    ts.tv_sec = mktime(&tm);
    clock_settime(CLOCK_REALTIME, &ts);  // tsにset
    printf("%d/%02d/%02d %02d:%02d:%02d.%09ld,%ld\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec, ts.tv_sec);

    for(;;) {
        fd_csv = fopen(csvfname, "a+");  // とりあえずひたすら追記する設定
        if(fd_csv == 0) {
            printf("open err(%s)\n", csvfname);
            return false;
        }
        // ret = sigtimedwait(&mask, NULL, &ts);
        // ret = sigwaitinfo(&mask, NULL);

        /*
        if(ret < 0) {
            printf("read error\n");
        } else if(ret != sizeof(posdat)) {
            ret = ERROR;
            printf("read size error\n");
        } else {
            ret = OK;
        }

        if(temp % 100 == 1) {
            printf(">Hour:%d, minute:%d, sec:%d, usec:%ld\n", posdat.receiver.time.hour, posdat.receiver.time.minute, posdat.receiver.time.sec, posdat.receiver.time.usec);
        }
        */
        /* Print POS data. */

        /* Print time. */

        // printf("ret: %d,%d\n", ret, errno);
        /*
        ret = sigtimedwait(&mask, NULL, &ts);
        if(ret == MY_GNSS_SIG) {
            ret = read_and_print(fd_gnss);
            if(ret < 0) {
                break;
            }
        } else {
            if(errno != 11) {
                printf("sigwaitinfo error %d, %d\n", ret, errno);
                break;
            } else {
                // printf("errno%d\n", errno);
            }
        }
        */
        temp++;
        temp %= 10000;
        if(temp % 1000 == 1) {
            ret = sigtimedwait(&mask, NULL, &tszero);
            // printf("sigwaitinfo start\n");
            // ret = sigwaitinfo(&mask, NULL);
            printf("sigtimedwait %d, %d\n", ret, errno);
            ret = read(fd_gnss, &posdat, sizeof(posdat));
            printf(">%d, %d/%d/%d Hour:%d, minute:%d, sec:%d, usec:%ld\n", temp - 20000, posdat.receiver.date.year, posdat.receiver.date.month, posdat.receiver.date.day, posdat.receiver.time.hour, posdat.receiver.time.minute, posdat.receiver.time.sec, posdat.receiver.time.usec);

            clock_gettime(CLOCK_REALTIME, &ts);  // tsをget
            localtime_r(&ts.tv_sec, &tm);        // tsをtmに格納（nsがほしい）

            time_diff = (tm.tm_sec * 1000 + (ts.tv_nsec) / 1000000) % 60000 - (posdat.receiver.time.sec * 1000 + posdat.receiver.time.usec / 1000) % 60000;
            if((time_diff >= 60000 || time_diff <= -60000) || ((int)(posdat.receiver.date.year) - 1900 > tm.tm_year)) {  // 秒数差が1以上と年に差がでたら修正
                // tsをget
                clock_gettime(CLOCK_REALTIME, &ts);
                // tsをtmに格納（nsだけがほしい）
                localtime_r(&ts.tv_sec, &tm);
                // GPSのほうからtmにデータ格納
                tm.tm_year = posdat.receiver.date.year - 1900;
                tm.tm_mon = posdat.receiver.date.month - 1;
                tm.tm_mday = posdat.receiver.date.day;
                tm.tm_hour = posdat.receiver.time.hour;
                tm.tm_min = posdat.receiver.time.minute;
                tm.tm_sec = posdat.receiver.time.sec;
                ts.tv_sec = mktime(&tm);             // tmをtsに変換（gps情報が入る）
                clock_settime(CLOCK_REALTIME, &ts);  // tsにset
            }

            printf("%d/%02d/%02d %02d:%02d:%02d.%09ld,%ld\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec, ts.tv_sec);

            if(posdat.receiver.pos_fixmode != CXD56_GNSS_PVT_POSFIX_INVALID) {
                /* 2D fix or 3D fix.
                 * Convert latitude and longitude into dmf format and print it. */

                double_to_dmf(posdat.receiver.latitude, &dmf);
                printf(">LAT %d.%d.%04ld\n", dmf.degree, dmf.minute, dmf.frac);

                double_to_dmf(posdat.receiver.longitude, &dmf);
                printf(">LNG %d.%d.%04ld\n", dmf.degree, dmf.minute, dmf.frac);
            }
        }

        ssize_t nbytes = read(fd, buftop, BUFSIZE);

        if(nbytes < 0) {
            errval = errno;
            printf("read failed:%d\n", errval);
            return;
        }

        char *start = buftop;
        char *end = buftop + BUFSIZE;

        logs[0] = (int16_t)(*(uint16_t *)(start));

        for(;;) {
            sample = (int16_t)(*(uint16_t *)(start));  // sampleされた電圧の生値を格納
            start += sizeof(uint16_t);
            if(start >= end) {  // endにいくまで配列の値を読み続ける
                break;
            }
            if(diff > 500) {
                // if(sample > -25000) {
                kiroku = counter;
                flag = true;
            }

            if(flag & (counter == (kiroku + plot_center) % BUFSIZE)) {
                struct tm tm_jst;
                time_t tv_sec_jst = ts.tv_sec + TIMEZONE_JST;
                clock_gettime(CLOCK_REALTIME, &ts);
                localtime_r(&tv_sec_jst, &tm_jst);
                printf("%d/%02d/%02d %02d:%02d:%02d.%06ld\n", tm_jst.tm_year + 1900, tm_jst.tm_mon + 1, tm_jst.tm_mday, tm_jst.tm_hour, tm_jst.tm_min, tm_jst.tm_sec, ts.tv_nsec / 1000);
                // YMDhms,microsec
                fprintf(fd_csv, "%d,%d,%d,%d,%d,%d,%d", tm_jst.tm_year + 1900, tm_jst.tm_mon + 1, tm_jst.tm_mday, tm_jst.tm_hour, tm_jst.tm_min, tm_jst.tm_sec, ts.tv_nsec / 1000);
                for(int i = 0; i < plot_center * 2; i++) {
                    fprintf(fd_csv, ",%d", logs[(i + kiroku - plot_center) % BUFSIZE]);
                }
                fprintf(fd_csv, "\n");
                flag = false;
            }
            counter++;
            counter %= BUFSIZE;
            logs[counter] = sample;
            diff = logs[counter] - logs[counter - 1];

            // printf("%d,%d,%d\n", sample, -20800, -28800);
        }
        fclose(fd_csv);
    }

    /* close CSV */

    fclose(fd_csv);

    /* Stop A/D conversion */

    ret = ioctl(fd, ANIOC_CXD56_STOP, 0);
    if(ret < 0) {
        int errcode = errno;
        printf("ioctl(STOP) failed: %d\n", errcode);
    }

    close(fd);

    /* Stop GNSS */

    ret = ioctl(fd_gnss, CXD56_GNSS_IOCTL_STOP, 0);
    if(ret < 0) {
        printf("stop GNSS ERROR\n");
    } else {
        printf("stop GNSS OK\n");
    }

    printf("\n");
    return OK;

    /* Error exits */

errout_with_dev:
    printf("ADC monitor example terminating! errout_with_dev\n");
    close(fd);

errout:
    printf("ADC monitor example terminating!\n");
    fflush(stdout);
    return errval;

_err:
    return -1;
}