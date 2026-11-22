#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
/* body = '$'와 '*' 사이 전부 (종류 글자 포함) */
uint8_t frame_xor(const char *body, size_t len) {
    uint8_t cs = 0;
    while (len--) cs ^= (uint8_t)*body++;
    return cs;
}
/* out에 "$body*HH\n" 작성. 반환: 길이, 공간 부족이면 0 */
int frame_build(char *out, size_t cap, const char *body) {
    int n = snprintf(out, cap, "$%s*%02X\n", body, frame_xor(body, strlen(body)));
    return (n > 0 && (size_t)n < cap) ? n : 0;
}
/* line = "$....*HH" (개행 제거된 상태). 맞으면 1, body 시작/길이를 돌려줌 */
int frame_check(const char *line, size_t len, const char **body, size_t *blen) {
    if (len < 5 || line[0] != '$' || line[len - 3] != '*') return 0;
    char hex[3] = { line[len - 2], line[len - 1], 0 };
    char *end;
    unsigned long want = strtoul(hex, &end, 16);
    if (end != hex + 2) return 0;
    if (frame_xor(line + 1, len - 4) != (uint8_t)want) return 0;
    *body = line + 1; *blen = len - 4;
    return 1;
}
int main(void) {
    const char *v[] = {"$R{\"n\":42,\"k\":\"DONE\"}*7B","$C,re=42,ok=1*67",
      "$T{\"n\":41,\"k\":\"TAG\",\"uid\":\"04A1B2C3\"}*36","$H{\"up\":123,\"st\":\"WORK\"}*51",
      "$S,st=WORK,lamp=Yb,fault=0*35","$X,id=7,in=S1,v=1*51","$C,re=42,ok=1*68","$C,re=42,ok=1*6G"};
    for (int i = 0; i < 8; i++) { const char *b; size_t bl;
      printf("%s -> %d\n", v[i], frame_check(v[i], strlen(v[i]), &b, &bl)); }
    char o[64]; frame_build(o, sizeof o, "L,up=0"); printf("%s", o);
    frame_build(o, sizeof o, "L,up=1"); printf("%s", o);
    return 0;
}
