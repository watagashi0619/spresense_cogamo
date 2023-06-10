#include <errno.h>          // error用
#include <fcntl.h>          // ファイルディスクリプタfdの操作。O_RDONLYとかの定義も。
#include <nuttx/config.h>   // config読み込み
#include <stdio.h>          // 標準入出力。printfとか
#include <stdlib.h>
#include <sys/boardctl.h>   // boardctl用。
#include <sys/ioctl.h>      // ioctl、電圧受取部の制御に必要。
#include <unistd.h>         // unix標準入出力。readとか。

#ifdef CONFIG_CXD56_ADC     // CONFIG_CXD56_ADCが宣言されている場合にブロック内を実行
#include <arch/chip/adc.h>  // ANIOC_CXD56_START,STOP
#include <arch/chip/scu.h>  // SCUIOC_SETFIFOMODE
#endif

#define BUFSIZE 8192                 // ADC値をよむfdに一度にどれだけ読ませるか？
#define ADC_DEVPATH "/dev/hpadc0"    // 読ませるADCピンの指定

#define CSVFILE_ROOTPATH "/mnt/sd0"  // SDカードのルートパス
#define MAX_PATH_LENGTH 128
static char csvfname[MAX_PATH_LENGTH];
static FILE *fd_csv;

char buf[BUFSIZE];
int fd;
int16_t sample;
int16_t logs[BUFSIZE];  // ここで宣言しないと0で初期化されない

int ret;
int errval;

// start ADC
static void start_adc() {
    fd = open(ADC_DEVPATH, O_RDONLY);        // ファイルディスクリプタからHPADC0を読む
    ret = ioctl(fd, SCUIOC_SETFIFOMODE, 1);  // 先入先出でfdに記録されるようにしている
    if(ret < 0) {
        errval = errno;
        printf("ioctl(SETFIFOMODE) failed: %d (%s)\n", errval, strerror(errval));
        goto _err;
    }
    ret = ioctl(fd, ANIOC_CXD56_START, 0);  // start
    if(ret < 0) {
        errval = errno;
        printf("ioctl(START) failed: %d (%s)\n", errval, strerror(errval));
        goto _err;
    }
_err:
    return;
}

// stop ADC
static void stop_adc() {
    ret = ioctl(fd, ANIOC_CXD56_STOP, 0);
    if(ret < 0) {
        errval = errno;
        printf("ioctl(STOP) failed: %d (%s)\n", errval, strerror(errval));
        goto _err;
    }
    close(fd);
_err:
    return;
}

// csvファイルをオープン
static void open_csv() {
    snprintf(csvfname, MAX_PATH_LENGTH, "%s/%s", CSVFILE_ROOTPATH, "log.csv");  // オープンするcsvファイルの名前を設定
    fd_csv = fopen(csvfname, "a+");                                             // とりあえずひたすら追記するとしてa+を設定
    if(fd_csv == 0) {
        printf("open err(%s)\n", csvfname);
        goto _err;
    }
_err:
    return;
}

// csvファイルを閉じる
static void close_csv() {
    fclose(fd_csv);
}

// loop
void loop() {
    int counter = 0;
    int entrypoint = 0;
    int interval = 50;
    int diff = 0;
    int threshold = 500;
    int diff_offset = 10;
    bool flag = false;

    char *buftop = &buf[0];  // bufの先頭ポインタを取得

    for(;;) {
        ssize_t nbytes = read(fd, buftop, BUFSIZE);
        char *start = buftop;
        char *end = buftop + BUFSIZE;

        for(;;) {
            sample = (int16_t)(*(uint16_t *)(start));  // sampleされた電圧の生値を格納
            start += sizeof(uint16_t);
            if(start >= end) {                         // endにいくまで配列の値を読み続ける
                break;
            }
            if(diff > threshold) {  // 記録開始地点を設定
                entrypoint = counter;
                flag = true;
            }
            if(flag && (counter == (entrypoint + interval / 2) % BUFSIZE)) {
                open_csv();                                                                   // csvをオープン
                fprintf(fd_csv, "%d", logs[(0 + entrypoint - interval / 2) % BUFSIZE]);       // csvファイルに書き込み
                for(int i = 1; i < interval; i++) {
                    fprintf(fd_csv, ",%d", logs[(i + entrypoint - interval / 2) % BUFSIZE]);  // csvファイルに書き込み
                }
                fprintf(fd_csv, "\n");
                close_csv();  // csvをクローズ
                flag = false;
            }
            counter++;
            counter %= BUFSIZE;
            logs[counter] = sample;
            diff = logs[counter] - logs[(counter - diff_offset) % BUFSIZE];
        }
    }
}

int main(int argc, FAR char *argv[]) {
    boardctl(BOARDIOC_INIT, 0);  // 本体の電源を入れるとこのアプリケーションが走るようにするための設定

    start_adc();                 // start ADC

    loop();

    stop_adc();  // stop ADC

    return 0;
}