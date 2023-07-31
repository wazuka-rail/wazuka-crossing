; pseudo instructions
skpz MACRO
    btfss ZERO
ENDM

skpnz MACRO
    btfsc ZERO
ENDM

skpc MACRO
    btfss CARRY
ENDM

skpnc MACRO
    btfsc CARRY
ENDM

movfw MACRO arg1
    movf arg1, W
ENDM


#define CONF1(reg, k, v, ...) (v << (reg##_##k##_POSITION))
#define CONF2(reg, k, v, ...) (v << (reg##_##k##_POSITION)) | CONF1(reg, ##__VA_ARGS__)
#define CONF3(reg, k, v, ...) (v << (reg##_##k##_POSITION)) | CONF2(reg, ##__VA_ARGS__)
#define CONF4(reg, k, v, ...) (v << (reg##_##k##_POSITION)) | CONF3(reg, ##__VA_ARGS__)
#define CONF5(reg, k, v, ...) (v << (reg##_##k##_POSITION)) | CONF4(reg, ##__VA_ARGS__)
#define CONF6(reg, k, v, ...) (v << (reg##_##k##_POSITION)) | CONF5(reg, ##__VA_ARGS__)
#define CONF7(reg, k, v, ...) (v << (reg##_##k##_POSITION)) | CONF6(reg, ##__VA_ARGS__)
#define CONF8(reg, k, v, ...) (v << (reg##_##k##_POSITION)) | CONF7(reg, ##__VA_ARGS__)

#define CONF_NUM(k1, v1, k2, v2, k3, v3, k4, v4, k5, v5, k6, v6, k7, v7, k8, v8, n, ...) n

#define CONFN_(n) CONF##n
#define CONFN(n) CONFN_(n)

#define CONF(reg, ...) \
    CONFN(CONF_NUM(##__VA_ARGS__, 8, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1)) (reg, ##__VA_ARGS__)
