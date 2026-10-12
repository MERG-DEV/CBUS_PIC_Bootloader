# 1 "../main.c"
# 1 "<built-in>" 1
# 1 "<built-in>" 3
# 295 "<built-in>" 3
# 1 "<command line>" 1
# 1 "<built-in>" 2
# 1 "C:\\Program Files\\Microchip\\xc8\\v3.10\\pic\\include/language_support.h" 1 3
# 2 "<built-in>" 2
# 1 "../main.c" 2
# 165 "../main.c"
# 1 "C:\\Program Files\\Microchip\\xc8\\v3.10\\pic\\include/xc.h" 1 3
# 18 "C:\\Program Files\\Microchip\\xc8\\v3.10\\pic\\include/xc.h" 3
extern const char __xc8_OPTIM_SPEED;

extern double __fpnormalize(double);


# 1 "C:\\Program Files\\Microchip\\xc8\\v3.10\\pic\\include\\c99/xc8debug.h" 1 3



# 1 "C:\\Program Files\\Microchip\\xc8\\v3.10\\pic\\include\\c99/stdlib.h" 1 3



# 1 "C:\\Program Files\\Microchip\\xc8\\v3.10\\pic\\include\\c99/musl_xc8.h" 1 3
# 5 "C:\\Program Files\\Microchip\\xc8\\v3.10\\pic\\include\\c99/stdlib.h" 2 3





# 1 "C:\\Program Files\\Microchip\\xc8\\v3.10\\pic\\include\\c99/features.h" 1 3
# 11 "C:\\Program Files\\Microchip\\xc8\\v3.10\\pic\\include\\c99/stdlib.h" 2 3
# 21 "C:\\Program Files\\Microchip\\xc8\\v3.10\\pic\\include\\c99/stdlib.h" 3
# 1 "C:\\Program Files\\Microchip\\xc8\\v3.10\\pic\\include\\c99/bits/alltypes.h" 1 3
# 24 "C:\\Program Files\\Microchip\\xc8\\v3.10\\pic\\include\\c99/bits/alltypes.h" 3
typedef long int wchar_t;
# 128 "C:\\Program Files\\Microchip\\xc8\\v3.10\\pic\\include\\c99/bits/alltypes.h" 3
typedef unsigned size_t;
# 174 "C:\\Program Files\\Microchip\\xc8\\v3.10\\pic\\include\\c99/bits/alltypes.h" 3
typedef __int24 int24_t;
# 210 "C:\\Program Files\\Microchip\\xc8\\v3.10\\pic\\include\\c99/bits/alltypes.h" 3
typedef __uint24 uint24_t;
# 22 "C:\\Program Files\\Microchip\\xc8\\v3.10\\pic\\include\\c99/stdlib.h" 2 3

int atoi (const char *);
long atol (const char *);

long long atoll (const char *);

double atof (const char *);


float strtof (const char *restrict, char **restrict);
double strtod (const char *restrict, char **restrict);
long double strtold (const char *restrict, char **restrict);



long strtol (const char *restrict, char **restrict, int);
unsigned long strtoul (const char *restrict, char **restrict, int);

long long strtoll (const char *restrict, char **restrict, int);
unsigned long long strtoull (const char *restrict, char **restrict, int);


unsigned long __strtoxl(const char * s, char ** endptr, int base, char is_signed);

unsigned long long __strtoxll(const char * s, char ** endptr, int base, char is_signed);
# 55 "C:\\Program Files\\Microchip\\xc8\\v3.10\\pic\\include\\c99/stdlib.h" 3
int rand (void);
void srand (unsigned);

void *malloc (size_t);
void *calloc (size_t, size_t);
void *realloc (void *, size_t);
void free (void *);

          void abort (void);
int atexit (void (*) (void));
          void exit (int);
          void _Exit (int);

void *bsearch (const void *, const void *, size_t, size_t, int (*)(const void *, const void *));







__attribute__((nonreentrant)) void qsort (void *, size_t, size_t, int (*)(const void *, const void *));

int abs (int);
long labs (long);

long long llabs (long long);


typedef struct { int quot, rem; } div_t;
typedef struct { long quot, rem; } ldiv_t;

typedef struct { long long quot, rem; } lldiv_t;


div_t div (int, int);
ldiv_t ldiv (long, long);

lldiv_t lldiv (long long, long long);


typedef struct { unsigned int quot, rem; } udiv_t;
typedef struct { unsigned long quot, rem; } uldiv_t;
udiv_t udiv (unsigned int, unsigned int);
uldiv_t uldiv (unsigned long, unsigned long);
# 5 "C:\\Program Files\\Microchip\\xc8\\v3.10\\pic\\include\\c99/xc8debug.h" 2 3







#pragma intrinsic(__builtin_software_breakpoint)
extern void __builtin_software_breakpoint(void);
# 24 "C:\\Program Files\\Microchip\\xc8\\v3.10\\pic\\include/xc.h" 2 3
# 1 "C:\\Program Files\\Microchip\\xc8\\v3.10\\pic\\include/builtins.h" 1 3



# 1 "C:\\Program Files\\Microchip\\xc8\\v3.10\\pic\\include\\c99/stdint.h" 1 3
# 26 "C:\\Program Files\\Microchip\\xc8\\v3.10\\pic\\include\\c99/stdint.h" 3
# 1 "C:\\Program Files\\Microchip\\xc8\\v3.10\\pic\\include\\c99/bits/alltypes.h" 1 3
# 133 "C:\\Program Files\\Microchip\\xc8\\v3.10\\pic\\include\\c99/bits/alltypes.h" 3
typedef unsigned __int24 uintptr_t;
# 148 "C:\\Program Files\\Microchip\\xc8\\v3.10\\pic\\include\\c99/bits/alltypes.h" 3
typedef __int24 intptr_t;
# 164 "C:\\Program Files\\Microchip\\xc8\\v3.10\\pic\\include\\c99/bits/alltypes.h" 3
typedef signed char int8_t;




typedef short int16_t;
# 179 "C:\\Program Files\\Microchip\\xc8\\v3.10\\pic\\include\\c99/bits/alltypes.h" 3
typedef long int32_t;





typedef long long int64_t;
# 194 "C:\\Program Files\\Microchip\\xc8\\v3.10\\pic\\include\\c99/bits/alltypes.h" 3
typedef long long intmax_t;





typedef unsigned char uint8_t;




typedef unsigned short uint16_t;
# 215 "C:\\Program Files\\Microchip\\xc8\\v3.10\\pic\\include\\c99/bits/alltypes.h" 3
typedef unsigned long uint32_t;





typedef unsigned long long uint64_t;
# 235 "C:\\Program Files\\Microchip\\xc8\\v3.10\\pic\\include\\c99/bits/alltypes.h" 3
typedef unsigned long long uintmax_t;
# 27 "C:\\Program Files\\Microchip\\xc8\\v3.10\\pic\\include\\c99/stdint.h" 2 3

typedef int8_t int_fast8_t;

typedef int64_t int_fast64_t;


typedef int8_t int_least8_t;
typedef int16_t int_least16_t;

typedef int24_t int_least24_t;
typedef int24_t int_fast24_t;

typedef int32_t int_least32_t;

typedef int64_t int_least64_t;


typedef uint8_t uint_fast8_t;

typedef uint64_t uint_fast64_t;


typedef uint8_t uint_least8_t;
typedef uint16_t uint_least16_t;

typedef uint24_t uint_least24_t;
typedef uint24_t uint_fast24_t;

typedef uint32_t uint_least32_t;

typedef uint64_t uint_least64_t;
# 148 "C:\\Program Files\\Microchip\\xc8\\v3.10\\pic\\include\\c99/stdint.h" 3
# 1 "C:\\Program Files\\Microchip\\xc8\\v3.10\\pic\\include\\c99/bits/stdint.h" 1 3
typedef int16_t int_fast16_t;
typedef int32_t int_fast32_t;
typedef uint16_t uint_fast16_t;
typedef uint32_t uint_fast32_t;
# 149 "C:\\Program Files\\Microchip\\xc8\\v3.10\\pic\\include\\c99/stdint.h" 2 3
# 5 "C:\\Program Files\\Microchip\\xc8\\v3.10\\pic\\include/builtins.h" 2 3


#pragma intrinsic(__nop)
extern void __nop(void);

#pragma intrinsic(__nopf000)
extern void __nopf000(void);
#pragma intrinsic(__nopffff)
extern void __nopffff(void);
#pragma intrinsic(__nop0000)
extern void __nop0000(void);



#pragma intrinsic(_delay)
extern __attribute__((nonreentrant)) void _delay(uint32_t);
#pragma intrinsic(_delaywdt)
extern __attribute__((nonreentrant)) void _delaywdt(uint32_t);

#pragma intrinsic(_delay3)
extern __attribute__((nonreentrant)) void _delay3(uint8_t);
# 25 "C:\\Program Files\\Microchip\\xc8\\v3.10\\pic\\include/xc.h" 2 3








# 1 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include/pic18.h" 1 3



# 1 "C:\\Program Files\\Microchip\\xc8\\v3.10\\pic\\include/xc.h" 1 3
# 5 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include/pic18.h" 2 3
# 1 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include/pic18_chip_select.h" 1 3
# 140 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include/pic18_chip_select.h" 3
# 1 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 1 3
# 36 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXERRCNT __attribute__((address(0xE41)));

__asm("RXERRCNT equ 0E41h");


typedef union {
    struct {
        unsigned int REC :8;
    };
    struct {
        unsigned int REC0 :1;
        unsigned int REC1 :1;
        unsigned int REC2 :1;
        unsigned int REC3 :1;
        unsigned int REC4 :1;
        unsigned int REC5 :1;
        unsigned int REC6 :1;
        unsigned int REC7 :1;
    };
} RXERRCNTbits_t;
extern volatile RXERRCNTbits_t RXERRCNTbits __attribute__((address(0xE41)));
# 106 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXERRCNT __attribute__((address(0xE42)));

__asm("TXERRCNT equ 0E42h");


typedef union {
    struct {
        unsigned int TEC :8;
    };
    struct {
        unsigned int TEC0 :1;
        unsigned int TEC1 :1;
        unsigned int TEC2 :1;
        unsigned int TEC3 :1;
        unsigned int TEC4 :1;
        unsigned int TEC5 :1;
        unsigned int TEC6 :1;
        unsigned int TEC7 :1;
    };
} TXERRCNTbits_t;
extern volatile TXERRCNTbits_t TXERRCNTbits __attribute__((address(0xE42)));
# 176 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char BRGCON1 __attribute__((address(0xE43)));

__asm("BRGCON1 equ 0E43h");


typedef union {
    struct {
        unsigned int BRP :6;
        unsigned int SJW :2;
    };
    struct {
        unsigned int BRP0 :1;
        unsigned int BRP1 :1;
        unsigned int BRP2 :1;
        unsigned int BRP3 :1;
        unsigned int BRP4 :1;
        unsigned int BRP5 :1;
        unsigned int SJW0 :1;
        unsigned int SJW1 :1;
    };
} BRGCON1bits_t;
extern volatile BRGCON1bits_t BRGCON1bits __attribute__((address(0xE43)));
# 252 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char BRGCON2 __attribute__((address(0xE44)));

__asm("BRGCON2 equ 0E44h");


typedef union {
    struct {
        unsigned int PRSEG :3;
        unsigned int SEG1PH :3;
        unsigned int SAM :1;
        unsigned int SEG2PHTS :1;
    };
    struct {
        unsigned int PRSEG0 :1;
        unsigned int PRSEG1 :1;
        unsigned int PRSEG2 :1;
        unsigned int SEG1PH0 :1;
        unsigned int SEG1PH1 :1;
        unsigned int SEG1PH2 :1;
    };
    struct {
        unsigned int :7;
        unsigned int SEG2PHT :1;
    };
} BRGCON2bits_t;
extern volatile BRGCON2bits_t BRGCON2bits __attribute__((address(0xE44)));
# 337 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char BRGCON3 __attribute__((address(0xE45)));

__asm("BRGCON3 equ 0E45h");


typedef union {
    struct {
        unsigned int SEG2PH :3;
        unsigned int :3;
        unsigned int WAKFIL :1;
        unsigned int WAKDIS :1;
    };
    struct {
        unsigned int SEG2PH0 :1;
        unsigned int SEG2PH1 :1;
        unsigned int SEG2PH2 :1;
    };
} BRGCON3bits_t;
extern volatile BRGCON3bits_t BRGCON3bits __attribute__((address(0xE45)));
# 390 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXFCON0 __attribute__((address(0xE46)));

__asm("RXFCON0 equ 0E46h");


typedef union {
    struct {
        unsigned int RXF0EN :1;
        unsigned int RXF1EN :1;
        unsigned int RXF2EN :1;
        unsigned int RXF3EN :1;
        unsigned int RXF4EN :1;
        unsigned int RXF5EN :1;
        unsigned int RXF6EN :1;
        unsigned int RXF7EN :1;
    };
} RXFCON0bits_t;
extern volatile RXFCON0bits_t RXFCON0bits __attribute__((address(0xE46)));
# 452 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXFCON1 __attribute__((address(0xE47)));

__asm("RXFCON1 equ 0E47h");


typedef union {
    struct {
        unsigned int RXF8EN :1;
        unsigned int RXF9EN :1;
        unsigned int RXF10EN :1;
        unsigned int RXF11EN :1;
        unsigned int RXF12EN :1;
        unsigned int RXF13EN :1;
        unsigned int RXF14EN :1;
        unsigned int RXF15EN :1;
    };
} RXFCON1bits_t;
extern volatile RXFCON1bits_t RXFCON1bits __attribute__((address(0xE47)));
# 514 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF6SIDH __attribute__((address(0xE48)));

__asm("RXF6SIDH equ 0E48h");


typedef union {
    struct {
        unsigned int SID :8;
    };
    struct {
        unsigned int SID3 :1;
        unsigned int SID4 :1;
        unsigned int SID5 :1;
        unsigned int SID6 :1;
        unsigned int SID7 :1;
        unsigned int SID8 :1;
        unsigned int SID9 :1;
        unsigned int SID10 :1;
    };
    struct {
        unsigned int RXF6SID3 :1;
        unsigned int RXF6SID4 :1;
        unsigned int RXF6SID5 :1;
        unsigned int RXF6SID6 :1;
        unsigned int RXF6SID7 :1;
        unsigned int RXF6SID8 :1;
        unsigned int RXF6SID9 :1;
        unsigned int RXF6SID10 :1;
    };
} RXF6SIDHbits_t;
extern volatile RXF6SIDHbits_t RXF6SIDHbits __attribute__((address(0xE48)));
# 634 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF6SIDL __attribute__((address(0xE49)));

__asm("RXF6SIDL equ 0E49h");


typedef union {
    struct {
        unsigned int EID :2;
        unsigned int :1;
        unsigned int EXIDEN :1;
        unsigned int :1;
        unsigned int SID :3;
    };
    struct {
        unsigned int EID16 :1;
        unsigned int EID17 :1;
        unsigned int :3;
        unsigned int SID0 :1;
        unsigned int SID1 :1;
        unsigned int SID2 :1;
    };
    struct {
        unsigned int RXF6EID16 :1;
        unsigned int RXF6EID17 :1;
        unsigned int :1;
        unsigned int RXF6EXIDEN :1;
        unsigned int :1;
        unsigned int RXF6SID0 :1;
        unsigned int RXF6SID1 :1;
        unsigned int RXF6SID2 :1;
    };
} RXF6SIDLbits_t;
extern volatile RXF6SIDLbits_t RXF6SIDLbits __attribute__((address(0xE49)));
# 741 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF6EIDH __attribute__((address(0xE4A)));

__asm("RXF6EIDH equ 0E4Ah");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID8 :1;
        unsigned int EID9 :1;
        unsigned int EID10 :1;
        unsigned int EID11 :1;
        unsigned int EID12 :1;
        unsigned int EID13 :1;
        unsigned int EID14 :1;
        unsigned int EID15 :1;
    };
    struct {
        unsigned int RXF6EID8 :1;
        unsigned int RXF6EID9 :1;
        unsigned int RXF6EID10 :1;
        unsigned int RXF6EID11 :1;
        unsigned int RXF6EID12 :1;
        unsigned int RXF6EID13 :1;
        unsigned int RXF6EID14 :1;
        unsigned int RXF6EID15 :1;
    };
} RXF6EIDHbits_t;
extern volatile RXF6EIDHbits_t RXF6EIDHbits __attribute__((address(0xE4A)));
# 861 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF6EIDL __attribute__((address(0xE4B)));

__asm("RXF6EIDL equ 0E4Bh");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID0 :1;
        unsigned int EID1 :1;
        unsigned int EID2 :1;
        unsigned int EID3 :1;
        unsigned int EID4 :1;
        unsigned int EID5 :1;
        unsigned int EID6 :1;
        unsigned int EID7 :1;
    };
    struct {
        unsigned int RXF6EID0 :1;
        unsigned int RXF6EID1 :1;
        unsigned int RXF6EID2 :1;
        unsigned int RXF6EID3 :1;
        unsigned int RXF6EID4 :1;
        unsigned int RXF6EID5 :1;
        unsigned int RXF6EID6 :1;
        unsigned int RXF6EID7 :1;
    };
} RXF6EIDLbits_t;
extern volatile RXF6EIDLbits_t RXF6EIDLbits __attribute__((address(0xE4B)));
# 981 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF7SIDH __attribute__((address(0xE4C)));

__asm("RXF7SIDH equ 0E4Ch");


typedef union {
    struct {
        unsigned int SID :8;
    };
    struct {
        unsigned int SID3 :1;
        unsigned int SID4 :1;
        unsigned int SID5 :1;
        unsigned int SID6 :1;
        unsigned int SID7 :1;
        unsigned int SID8 :1;
        unsigned int SID9 :1;
        unsigned int SID10 :1;
    };
    struct {
        unsigned int RXF7SID3 :1;
        unsigned int RXF7SID4 :1;
        unsigned int RXF7SID5 :1;
        unsigned int RXF7SID6 :1;
        unsigned int RXF7SID7 :1;
        unsigned int RXF7SID8 :1;
        unsigned int RXF7SID9 :1;
        unsigned int RXF7SID10 :1;
    };
} RXF7SIDHbits_t;
extern volatile RXF7SIDHbits_t RXF7SIDHbits __attribute__((address(0xE4C)));
# 1101 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF7SIDL __attribute__((address(0xE4D)));

__asm("RXF7SIDL equ 0E4Dh");


typedef union {
    struct {
        unsigned int EID :2;
        unsigned int :1;
        unsigned int EXIDEN :1;
        unsigned int :1;
        unsigned int SID :3;
    };
    struct {
        unsigned int EID16 :1;
        unsigned int EID17 :1;
        unsigned int :3;
        unsigned int SID0 :1;
        unsigned int SID1 :1;
        unsigned int SID2 :1;
    };
    struct {
        unsigned int RXF7EID16 :1;
        unsigned int RXF7EID17 :1;
        unsigned int :1;
        unsigned int RXF7EXIDEN :1;
        unsigned int :1;
        unsigned int RXF7SID0 :1;
        unsigned int RXF7SID1 :1;
        unsigned int RXF7SID2 :1;
    };
} RXF7SIDLbits_t;
extern volatile RXF7SIDLbits_t RXF7SIDLbits __attribute__((address(0xE4D)));
# 1208 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF7EIDH __attribute__((address(0xE4E)));

__asm("RXF7EIDH equ 0E4Eh");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID8 :1;
        unsigned int EID9 :1;
        unsigned int EID10 :1;
        unsigned int EID11 :1;
        unsigned int EID12 :1;
        unsigned int EID13 :1;
        unsigned int EID14 :1;
        unsigned int EID15 :1;
    };
    struct {
        unsigned int RXF7EID8 :1;
        unsigned int RXF7EID9 :1;
        unsigned int RXF7EID10 :1;
        unsigned int RXF7EID11 :1;
        unsigned int RXF7EID12 :1;
        unsigned int RXF7EID13 :1;
        unsigned int RXF7EID14 :1;
        unsigned int RXF7EID15 :1;
    };
} RXF7EIDHbits_t;
extern volatile RXF7EIDHbits_t RXF7EIDHbits __attribute__((address(0xE4E)));
# 1328 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF7EIDL __attribute__((address(0xE4F)));

__asm("RXF7EIDL equ 0E4Fh");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID0 :1;
        unsigned int EID1 :1;
        unsigned int EID2 :1;
        unsigned int EID3 :1;
        unsigned int EID4 :1;
        unsigned int EID5 :1;
        unsigned int EID6 :1;
        unsigned int EID7 :1;
    };
    struct {
        unsigned int RXF7EID0 :1;
        unsigned int RXF7EID1 :1;
        unsigned int RXF7EID2 :1;
        unsigned int RXF7EID3 :1;
        unsigned int RXF7EID4 :1;
        unsigned int RXF7EID5 :1;
        unsigned int RXF7EID6 :1;
        unsigned int RXF7EID7 :1;
    };
} RXF7EIDLbits_t;
extern volatile RXF7EIDLbits_t RXF7EIDLbits __attribute__((address(0xE4F)));
# 1448 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF8SIDH __attribute__((address(0xE50)));

__asm("RXF8SIDH equ 0E50h");


typedef union {
    struct {
        unsigned int SID :8;
    };
    struct {
        unsigned int SID3 :1;
        unsigned int SID4 :1;
        unsigned int SID5 :1;
        unsigned int SID6 :1;
        unsigned int SID7 :1;
        unsigned int SID8 :1;
        unsigned int SID9 :1;
        unsigned int SID10 :1;
    };
    struct {
        unsigned int RXF8SID3 :1;
        unsigned int RXF8SID4 :1;
        unsigned int RXF8SID5 :1;
        unsigned int RXF8SID6 :1;
        unsigned int RXF8SID7 :1;
        unsigned int RXF8SID8 :1;
        unsigned int RXF8SID9 :1;
        unsigned int RXF8SID10 :1;
    };
} RXF8SIDHbits_t;
extern volatile RXF8SIDHbits_t RXF8SIDHbits __attribute__((address(0xE50)));
# 1568 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF8SIDL __attribute__((address(0xE51)));

__asm("RXF8SIDL equ 0E51h");


typedef union {
    struct {
        unsigned int EID :2;
        unsigned int :1;
        unsigned int EXIDEN :1;
        unsigned int :1;
        unsigned int SID :3;
    };
    struct {
        unsigned int EID16 :1;
        unsigned int EID17 :1;
        unsigned int :3;
        unsigned int SID0 :1;
        unsigned int SID1 :1;
        unsigned int SID2 :1;
    };
    struct {
        unsigned int RXF8EID16 :1;
        unsigned int RXF8EID17 :1;
        unsigned int :1;
        unsigned int RXF8EXIDEN :1;
        unsigned int :1;
        unsigned int RXF8SID0 :1;
        unsigned int RXF8SID1 :1;
        unsigned int RXF8SID2 :1;
    };
} RXF8SIDLbits_t;
extern volatile RXF8SIDLbits_t RXF8SIDLbits __attribute__((address(0xE51)));
# 1675 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF8EIDH __attribute__((address(0xE52)));

__asm("RXF8EIDH equ 0E52h");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID8 :1;
        unsigned int EID9 :1;
        unsigned int EID10 :1;
        unsigned int EID11 :1;
        unsigned int EID12 :1;
        unsigned int EID13 :1;
        unsigned int EID14 :1;
        unsigned int EID15 :1;
    };
    struct {
        unsigned int RXF8EID8 :1;
        unsigned int RXF8EID9 :1;
        unsigned int RXF8EID10 :1;
        unsigned int RXF8EID11 :1;
        unsigned int RXF8EID12 :1;
        unsigned int RXF8EID13 :1;
        unsigned int RXF8EID14 :1;
        unsigned int RXF8EID15 :1;
    };
} RXF8EIDHbits_t;
extern volatile RXF8EIDHbits_t RXF8EIDHbits __attribute__((address(0xE52)));
# 1795 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF8EIDL __attribute__((address(0xE53)));

__asm("RXF8EIDL equ 0E53h");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID0 :1;
        unsigned int EID1 :1;
        unsigned int EID2 :1;
        unsigned int EID3 :1;
        unsigned int EID4 :1;
        unsigned int EID5 :1;
        unsigned int EID6 :1;
        unsigned int EID7 :1;
    };
    struct {
        unsigned int RXF8EID0 :1;
        unsigned int RXF8EID1 :1;
        unsigned int RXF8EID2 :1;
        unsigned int RXF8EID3 :1;
        unsigned int RXF8EID4 :1;
        unsigned int RXF8EID5 :1;
        unsigned int RXF8EID6 :1;
        unsigned int RXF8EID7 :1;
    };
} RXF8EIDLbits_t;
extern volatile RXF8EIDLbits_t RXF8EIDLbits __attribute__((address(0xE53)));
# 1915 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF9SIDH __attribute__((address(0xE54)));

__asm("RXF9SIDH equ 0E54h");


typedef union {
    struct {
        unsigned int SID :8;
    };
    struct {
        unsigned int SID3 :1;
        unsigned int SID4 :1;
        unsigned int SID5 :1;
        unsigned int SID6 :1;
        unsigned int SID7 :1;
        unsigned int SID8 :1;
        unsigned int SID9 :1;
        unsigned int SID10 :1;
    };
    struct {
        unsigned int RXF9SID3 :1;
        unsigned int RXF9SID4 :1;
        unsigned int RXF9SID5 :1;
        unsigned int RXF9SID6 :1;
        unsigned int RXF9SID7 :1;
        unsigned int RXF9SID8 :1;
        unsigned int RXF9SID9 :1;
        unsigned int RXF9SID10 :1;
    };
} RXF9SIDHbits_t;
extern volatile RXF9SIDHbits_t RXF9SIDHbits __attribute__((address(0xE54)));
# 2035 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF9SIDL __attribute__((address(0xE55)));

__asm("RXF9SIDL equ 0E55h");


typedef union {
    struct {
        unsigned int EID :2;
        unsigned int :1;
        unsigned int EXIDEN :1;
        unsigned int :1;
        unsigned int SID :3;
    };
    struct {
        unsigned int EID16 :1;
        unsigned int EID17 :1;
        unsigned int :3;
        unsigned int SID0 :1;
        unsigned int SID1 :1;
        unsigned int SID2 :1;
    };
    struct {
        unsigned int RXF9EID16 :1;
        unsigned int RXF9EID17 :1;
        unsigned int :1;
        unsigned int RXF9EXIDEN :1;
        unsigned int :1;
        unsigned int RXF9SID0 :1;
        unsigned int RXF9SID1 :1;
        unsigned int RXF9SID2 :1;
    };
} RXF9SIDLbits_t;
extern volatile RXF9SIDLbits_t RXF9SIDLbits __attribute__((address(0xE55)));
# 2142 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF9EIDH __attribute__((address(0xE56)));

__asm("RXF9EIDH equ 0E56h");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID8 :1;
        unsigned int EID9 :1;
        unsigned int EID10 :1;
        unsigned int EID11 :1;
        unsigned int EID12 :1;
        unsigned int EID13 :1;
        unsigned int EID14 :1;
        unsigned int EID15 :1;
    };
    struct {
        unsigned int RXF9EID8 :1;
        unsigned int RXF9EID9 :1;
        unsigned int RXF9EID10 :1;
        unsigned int RXF9EID11 :1;
        unsigned int RXF9EID12 :1;
        unsigned int RXF9EID13 :1;
        unsigned int RXF9EID14 :1;
        unsigned int RXF9EID15 :1;
    };
} RXF9EIDHbits_t;
extern volatile RXF9EIDHbits_t RXF9EIDHbits __attribute__((address(0xE56)));
# 2262 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF9EIDL __attribute__((address(0xE57)));

__asm("RXF9EIDL equ 0E57h");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID0 :1;
        unsigned int EID1 :1;
        unsigned int EID2 :1;
        unsigned int EID3 :1;
        unsigned int EID4 :1;
        unsigned int EID5 :1;
        unsigned int EID6 :1;
        unsigned int EID7 :1;
    };
    struct {
        unsigned int RXF9EID0 :1;
        unsigned int RXF9EID1 :1;
        unsigned int RXF9EID2 :1;
        unsigned int RXF9EID3 :1;
        unsigned int RXF9EID4 :1;
        unsigned int RXF9EID5 :1;
        unsigned int RXF9EID6 :1;
        unsigned int RXF9EID7 :1;
    };
} RXF9EIDLbits_t;
extern volatile RXF9EIDLbits_t RXF9EIDLbits __attribute__((address(0xE57)));
# 2382 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF10SIDH __attribute__((address(0xE58)));

__asm("RXF10SIDH equ 0E58h");


typedef union {
    struct {
        unsigned int SID :8;
    };
    struct {
        unsigned int SID3 :1;
        unsigned int SID4 :1;
        unsigned int SID5 :1;
        unsigned int SID6 :1;
        unsigned int SID7 :1;
        unsigned int SID8 :1;
        unsigned int SID9 :1;
        unsigned int SID10 :1;
    };
    struct {
        unsigned int RXF10SID3 :1;
        unsigned int RXF10SID4 :1;
        unsigned int RXF10SID5 :1;
        unsigned int RXF10SID6 :1;
        unsigned int RXF10SID7 :1;
        unsigned int RXF10SID8 :1;
        unsigned int RXF10SID9 :1;
        unsigned int RXF10SID10 :1;
    };
} RXF10SIDHbits_t;
extern volatile RXF10SIDHbits_t RXF10SIDHbits __attribute__((address(0xE58)));
# 2502 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF10SIDL __attribute__((address(0xE59)));

__asm("RXF10SIDL equ 0E59h");


typedef union {
    struct {
        unsigned int EID :2;
        unsigned int :1;
        unsigned int EXIDEN :1;
        unsigned int :1;
        unsigned int SID :3;
    };
    struct {
        unsigned int EID16 :1;
        unsigned int EID17 :1;
        unsigned int :3;
        unsigned int SID0 :1;
        unsigned int SID1 :1;
        unsigned int SID2 :1;
    };
    struct {
        unsigned int RXF10EID16 :1;
        unsigned int RXF10EID17 :1;
        unsigned int :1;
        unsigned int RXF10EXIDEN :1;
        unsigned int :1;
        unsigned int RXF10SID0 :1;
        unsigned int RXF10SID1 :1;
        unsigned int RXF10SID2 :1;
    };
} RXF10SIDLbits_t;
extern volatile RXF10SIDLbits_t RXF10SIDLbits __attribute__((address(0xE59)));
# 2609 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF10EIDH __attribute__((address(0xE5A)));

__asm("RXF10EIDH equ 0E5Ah");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID8 :1;
        unsigned int EID9 :1;
        unsigned int EID10 :1;
        unsigned int EID11 :1;
        unsigned int EID12 :1;
        unsigned int EID13 :1;
        unsigned int EID14 :1;
        unsigned int EID15 :1;
    };
    struct {
        unsigned int RXF10EID8 :1;
        unsigned int RXF10EID9 :1;
        unsigned int RXF10EID10 :1;
        unsigned int RXF10EID11 :1;
        unsigned int RXF10EID12 :1;
        unsigned int RXF10EID13 :1;
        unsigned int RXF10EID14 :1;
        unsigned int RXF10EID15 :1;
    };
} RXF10EIDHbits_t;
extern volatile RXF10EIDHbits_t RXF10EIDHbits __attribute__((address(0xE5A)));
# 2729 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF10EIDL __attribute__((address(0xE5B)));

__asm("RXF10EIDL equ 0E5Bh");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID0 :1;
        unsigned int EID1 :1;
        unsigned int EID2 :1;
        unsigned int EID3 :1;
        unsigned int EID4 :1;
        unsigned int EID5 :1;
        unsigned int EID6 :1;
        unsigned int EID7 :1;
    };
    struct {
        unsigned int RXF10EID0 :1;
        unsigned int RXF10EID1 :1;
        unsigned int RXF10EID2 :1;
        unsigned int RXF10EID3 :1;
        unsigned int RXF10EID4 :1;
        unsigned int RXF10EID5 :1;
        unsigned int RXF10EID6 :1;
        unsigned int RXF10EID7 :1;
    };
} RXF10EIDLbits_t;
extern volatile RXF10EIDLbits_t RXF10EIDLbits __attribute__((address(0xE5B)));
# 2849 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF11SIDH __attribute__((address(0xE5C)));

__asm("RXF11SIDH equ 0E5Ch");


typedef union {
    struct {
        unsigned int RXFSID :8;
    };
    struct {
        unsigned int SID3 :1;
        unsigned int SID4 :1;
        unsigned int SID5 :1;
        unsigned int SID6 :1;
        unsigned int SID7 :1;
        unsigned int SID8 :1;
        unsigned int SID9 :1;
        unsigned int SID10 :1;
    };
    struct {
        unsigned int RXF11SID3 :1;
        unsigned int RXF11SID4 :1;
        unsigned int RXF11SID5 :1;
        unsigned int RXF11SID6 :1;
        unsigned int RXF11SID7 :1;
        unsigned int RXF11SID8 :1;
        unsigned int RXF11SID9 :1;
        unsigned int RXF11SID10 :1;
    };
} RXF11SIDHbits_t;
extern volatile RXF11SIDHbits_t RXF11SIDHbits __attribute__((address(0xE5C)));
# 2969 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF11SIDL __attribute__((address(0xE5D)));

__asm("RXF11SIDL equ 0E5Dh");


typedef union {
    struct {
        unsigned int EID :2;
        unsigned int :1;
        unsigned int EXIDEN :1;
        unsigned int :1;
        unsigned int SID :3;
    };
    struct {
        unsigned int EID16 :1;
        unsigned int EID17 :1;
        unsigned int :3;
        unsigned int SID0 :1;
        unsigned int SID1 :1;
        unsigned int SID2 :1;
    };
    struct {
        unsigned int RXF11EID16 :1;
        unsigned int RXF11EID17 :1;
        unsigned int :1;
        unsigned int RXF11EXIDEN :1;
        unsigned int :1;
        unsigned int RXF11SID0 :1;
        unsigned int RXF11SID1 :1;
        unsigned int RXF11SID2 :1;
    };
} RXF11SIDLbits_t;
extern volatile RXF11SIDLbits_t RXF11SIDLbits __attribute__((address(0xE5D)));
# 3076 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF11EIDH __attribute__((address(0xE5E)));

__asm("RXF11EIDH equ 0E5Eh");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID8 :1;
        unsigned int EID9 :1;
        unsigned int EID10 :1;
        unsigned int EID11 :1;
        unsigned int EID12 :1;
        unsigned int EID13 :1;
        unsigned int EID14 :1;
        unsigned int EID15 :1;
    };
    struct {
        unsigned int RXF11EID8 :1;
        unsigned int RXF11EID9 :1;
        unsigned int RXF11EID10 :1;
        unsigned int RXF11EID11 :1;
        unsigned int RXF11EID12 :1;
        unsigned int RXF11EID13 :1;
        unsigned int RXF11EID14 :1;
        unsigned int RXF11EID15 :1;
    };
} RXF11EIDHbits_t;
extern volatile RXF11EIDHbits_t RXF11EIDHbits __attribute__((address(0xE5E)));
# 3196 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF11EIDL __attribute__((address(0xE5F)));

__asm("RXF11EIDL equ 0E5Fh");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID0 :1;
        unsigned int EID1 :1;
        unsigned int EID2 :1;
        unsigned int EID3 :1;
        unsigned int EID4 :1;
        unsigned int EID5 :1;
        unsigned int EID6 :1;
        unsigned int EID7 :1;
    };
    struct {
        unsigned int RXF11EID0 :1;
        unsigned int RXF11EID1 :1;
        unsigned int RXF11EID2 :1;
        unsigned int RXF11EID3 :1;
        unsigned int RXF11EID4 :1;
        unsigned int RXF11EID5 :1;
        unsigned int RXF11EID6 :1;
        unsigned int RXF11EID7 :1;
    };
} RXF11EIDLbits_t;
extern volatile RXF11EIDLbits_t RXF11EIDLbits __attribute__((address(0xE5F)));
# 3316 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF12SIDH __attribute__((address(0xE60)));

__asm("RXF12SIDH equ 0E60h");


typedef union {
    struct {
        unsigned int SID :8;
    };
    struct {
        unsigned int SID3 :1;
        unsigned int SID4 :1;
        unsigned int SID5 :1;
        unsigned int SID6 :1;
        unsigned int SID7 :1;
        unsigned int SID8 :1;
        unsigned int SID9 :1;
        unsigned int SID10 :1;
    };
    struct {
        unsigned int RXF12SID3 :1;
        unsigned int RXF12SID4 :1;
        unsigned int RXF12SID5 :1;
        unsigned int RXF12SID6 :1;
        unsigned int RXF12SID7 :1;
        unsigned int RXF12SID8 :1;
        unsigned int RXF12SID9 :1;
        unsigned int RXF12SID10 :1;
    };
} RXF12SIDHbits_t;
extern volatile RXF12SIDHbits_t RXF12SIDHbits __attribute__((address(0xE60)));
# 3436 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF12SIDL __attribute__((address(0xE61)));

__asm("RXF12SIDL equ 0E61h");


typedef union {
    struct {
        unsigned int EID :2;
        unsigned int :1;
        unsigned int EXIDEN :1;
        unsigned int :1;
        unsigned int SID :3;
    };
    struct {
        unsigned int EID16 :1;
        unsigned int EID17 :1;
        unsigned int :3;
        unsigned int SID0 :1;
        unsigned int SID1 :1;
        unsigned int SID2 :1;
    };
    struct {
        unsigned int RXF12EID16 :1;
        unsigned int RXF12EID17 :1;
        unsigned int :1;
        unsigned int RXF12EXIDEN :1;
        unsigned int :1;
        unsigned int RXF12SID0 :1;
        unsigned int RXF12SID1 :1;
        unsigned int RXF12SID2 :1;
    };
} RXF12SIDLbits_t;
extern volatile RXF12SIDLbits_t RXF12SIDLbits __attribute__((address(0xE61)));
# 3543 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF12EIDH __attribute__((address(0xE62)));

__asm("RXF12EIDH equ 0E62h");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID8 :1;
        unsigned int EID9 :1;
        unsigned int EID10 :1;
        unsigned int EID11 :1;
        unsigned int EID12 :1;
        unsigned int EID13 :1;
        unsigned int EID14 :1;
        unsigned int EID15 :1;
    };
    struct {
        unsigned int RXF12EID8 :1;
        unsigned int RXF12EID9 :1;
        unsigned int RXF12EID10 :1;
        unsigned int RXF12EID11 :1;
        unsigned int RXF12EID12 :1;
        unsigned int RXF12EID13 :1;
        unsigned int RXF12EID14 :1;
        unsigned int RXF12EID15 :1;
    };
} RXF12EIDHbits_t;
extern volatile RXF12EIDHbits_t RXF12EIDHbits __attribute__((address(0xE62)));
# 3663 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF12EIDL __attribute__((address(0xE63)));

__asm("RXF12EIDL equ 0E63h");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID0 :1;
        unsigned int EID1 :1;
        unsigned int EID2 :1;
        unsigned int EID3 :1;
        unsigned int EID4 :1;
        unsigned int EID5 :1;
        unsigned int EID6 :1;
        unsigned int EID7 :1;
    };
    struct {
        unsigned int RXF12EID0 :1;
        unsigned int RXF12EID1 :1;
        unsigned int RXF12EID2 :1;
        unsigned int RXF12EID3 :1;
        unsigned int RXF12EID4 :1;
        unsigned int RXF12EID5 :1;
        unsigned int RXF12EID6 :1;
        unsigned int RXF12EID7 :1;
    };
} RXF12EIDLbits_t;
extern volatile RXF12EIDLbits_t RXF12EIDLbits __attribute__((address(0xE63)));
# 3783 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF13SIDH __attribute__((address(0xE64)));

__asm("RXF13SIDH equ 0E64h");


typedef union {
    struct {
        unsigned int SID :8;
    };
    struct {
        unsigned int SID3 :1;
        unsigned int SID4 :1;
        unsigned int SID5 :1;
        unsigned int SID6 :1;
        unsigned int SID7 :1;
        unsigned int SID8 :1;
        unsigned int SID9 :1;
        unsigned int SID10 :1;
    };
    struct {
        unsigned int RXF13SID3 :1;
        unsigned int RXF13SID4 :1;
        unsigned int RXF13SID5 :1;
        unsigned int RXF13SID6 :1;
        unsigned int RXF13SID7 :1;
        unsigned int RXF13SID8 :1;
        unsigned int RXF13SID9 :1;
        unsigned int RXF13SID10 :1;
    };
} RXF13SIDHbits_t;
extern volatile RXF13SIDHbits_t RXF13SIDHbits __attribute__((address(0xE64)));
# 3903 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF13SIDL __attribute__((address(0xE65)));

__asm("RXF13SIDL equ 0E65h");


typedef union {
    struct {
        unsigned int EID :2;
        unsigned int :1;
        unsigned int EXIDEN :1;
        unsigned int :1;
        unsigned int SID :3;
    };
    struct {
        unsigned int EID16 :1;
        unsigned int EID17 :1;
        unsigned int :3;
        unsigned int SID0 :1;
        unsigned int SID1 :1;
        unsigned int SID2 :1;
    };
    struct {
        unsigned int RXF13EID16 :1;
        unsigned int RXF13EID17 :1;
        unsigned int :1;
        unsigned int RXF13EXIDEN :1;
        unsigned int :1;
        unsigned int RXF13SID0 :1;
        unsigned int RXF13SID1 :1;
        unsigned int RXF13SID2 :1;
    };
} RXF13SIDLbits_t;
extern volatile RXF13SIDLbits_t RXF13SIDLbits __attribute__((address(0xE65)));
# 4010 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF13EIDH __attribute__((address(0xE66)));

__asm("RXF13EIDH equ 0E66h");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID8 :1;
        unsigned int EID9 :1;
        unsigned int EID10 :1;
        unsigned int EID11 :1;
        unsigned int EID12 :1;
        unsigned int EID13 :1;
        unsigned int EID14 :1;
        unsigned int EID15 :1;
    };
    struct {
        unsigned int RXF13EID8 :1;
        unsigned int RXF13EID9 :1;
        unsigned int RXF13EID10 :1;
        unsigned int RXF13EID11 :1;
        unsigned int RXF13EID12 :1;
        unsigned int RXF13EID13 :1;
        unsigned int RXF13EID14 :1;
        unsigned int RXF13EID15 :1;
    };
} RXF13EIDHbits_t;
extern volatile RXF13EIDHbits_t RXF13EIDHbits __attribute__((address(0xE66)));
# 4130 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF13EIDL __attribute__((address(0xE67)));

__asm("RXF13EIDL equ 0E67h");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID0 :1;
        unsigned int EID1 :1;
        unsigned int EID2 :1;
        unsigned int EID3 :1;
        unsigned int EID4 :1;
        unsigned int EID5 :1;
        unsigned int EID6 :1;
        unsigned int EID7 :1;
    };
    struct {
        unsigned int RXF13EID0 :1;
        unsigned int RXF13EID1 :1;
        unsigned int RXF13EID2 :1;
        unsigned int RXF13EID3 :1;
        unsigned int RXF13EID4 :1;
        unsigned int RXF13EID5 :1;
        unsigned int RXF13EID6 :1;
        unsigned int RXF13EID7 :1;
    };
} RXF13EIDLbits_t;
extern volatile RXF13EIDLbits_t RXF13EIDLbits __attribute__((address(0xE67)));
# 4250 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF14SIDH __attribute__((address(0xE68)));

__asm("RXF14SIDH equ 0E68h");


typedef union {
    struct {
        unsigned int SID :8;
    };
    struct {
        unsigned int SID3 :1;
        unsigned int SID4 :1;
        unsigned int SID5 :1;
        unsigned int SID6 :1;
        unsigned int SID7 :1;
        unsigned int SID8 :1;
        unsigned int SID9 :1;
        unsigned int SID10 :1;
    };
    struct {
        unsigned int RXF14SID3 :1;
        unsigned int RXF14SID4 :1;
        unsigned int RXF14SID5 :1;
        unsigned int RXF14SID6 :1;
        unsigned int RXF14SID7 :1;
        unsigned int RXF14SID8 :1;
        unsigned int RXF14SID9 :1;
        unsigned int RXF14SID10 :1;
    };
} RXF14SIDHbits_t;
extern volatile RXF14SIDHbits_t RXF14SIDHbits __attribute__((address(0xE68)));
# 4370 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF14SIDL __attribute__((address(0xE69)));

__asm("RXF14SIDL equ 0E69h");


typedef union {
    struct {
        unsigned int EID :2;
        unsigned int :1;
        unsigned int EXIDEN :1;
        unsigned int :1;
        unsigned int SID :3;
    };
    struct {
        unsigned int EID16 :1;
        unsigned int EID17 :1;
        unsigned int :3;
        unsigned int SID0 :1;
        unsigned int SID1 :1;
        unsigned int SID2 :1;
    };
    struct {
        unsigned int RXF14EID16 :1;
        unsigned int RXF14EID17 :1;
        unsigned int :1;
        unsigned int RXF14EXIDEN :1;
        unsigned int :1;
        unsigned int RXF14SID0 :1;
        unsigned int RXF14SID1 :1;
        unsigned int RXF14SID2 :1;
    };
} RXF14SIDLbits_t;
extern volatile RXF14SIDLbits_t RXF14SIDLbits __attribute__((address(0xE69)));
# 4477 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF14EIDH __attribute__((address(0xE6A)));

__asm("RXF14EIDH equ 0E6Ah");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID8 :1;
        unsigned int EID9 :1;
        unsigned int EID10 :1;
        unsigned int EID11 :1;
        unsigned int EID12 :1;
        unsigned int EID13 :1;
        unsigned int EID14 :1;
        unsigned int EID15 :1;
    };
    struct {
        unsigned int RXF14EID8 :1;
        unsigned int RXF14EID9 :1;
        unsigned int RXF14EID10 :1;
        unsigned int RXF14EID11 :1;
        unsigned int RXF14EID12 :1;
        unsigned int RXF14EID13 :1;
        unsigned int RXF14EID14 :1;
        unsigned int RXF14EID15 :1;
    };
} RXF14EIDHbits_t;
extern volatile RXF14EIDHbits_t RXF14EIDHbits __attribute__((address(0xE6A)));
# 4597 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF14EIDL __attribute__((address(0xE6B)));

__asm("RXF14EIDL equ 0E6Bh");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID0 :1;
        unsigned int EID1 :1;
        unsigned int EID2 :1;
        unsigned int EID3 :1;
        unsigned int EID4 :1;
        unsigned int EID5 :1;
        unsigned int EID6 :1;
        unsigned int EID7 :1;
    };
    struct {
        unsigned int RXF14EID0 :1;
        unsigned int RXF14EID1 :1;
        unsigned int RXF14EID2 :1;
        unsigned int RXF14EID3 :1;
        unsigned int RXF14EID4 :1;
        unsigned int RXF14EID5 :1;
        unsigned int RXF14EID6 :1;
        unsigned int RXF14EID7 :1;
    };
} RXF14EIDLbits_t;
extern volatile RXF14EIDLbits_t RXF14EIDLbits __attribute__((address(0xE6B)));
# 4717 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF15SIDH __attribute__((address(0xE6C)));

__asm("RXF15SIDH equ 0E6Ch");


typedef union {
    struct {
        unsigned int SID :8;
    };
    struct {
        unsigned int SID3 :1;
        unsigned int SID4 :1;
        unsigned int SID5 :1;
        unsigned int SID6 :1;
        unsigned int SID7 :1;
        unsigned int SID8 :1;
        unsigned int SID9 :1;
        unsigned int SID10 :1;
    };
    struct {
        unsigned int RXF15SID3 :1;
        unsigned int RXF15SID4 :1;
        unsigned int RXF15SID5 :1;
        unsigned int RXF15SID6 :1;
        unsigned int RXF15SID7 :1;
        unsigned int RXF15SID8 :1;
        unsigned int RXF15SID9 :1;
        unsigned int RXF15SID10 :1;
    };
} RXF15SIDHbits_t;
extern volatile RXF15SIDHbits_t RXF15SIDHbits __attribute__((address(0xE6C)));
# 4837 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF15SIDL __attribute__((address(0xE6D)));

__asm("RXF15SIDL equ 0E6Dh");


typedef union {
    struct {
        unsigned int EID :2;
        unsigned int :1;
        unsigned int EXIDEN :1;
        unsigned int :1;
        unsigned int SID :3;
    };
    struct {
        unsigned int EID16 :1;
        unsigned int EID17 :1;
        unsigned int :3;
        unsigned int SID0 :1;
        unsigned int SID1 :1;
        unsigned int SID2 :1;
    };
    struct {
        unsigned int RXF15EID16 :1;
        unsigned int RXF15EID17 :1;
        unsigned int :1;
        unsigned int RXF15EXIDEN :1;
        unsigned int :1;
        unsigned int RXF15SID0 :1;
        unsigned int RXF15SID1 :1;
        unsigned int RXF15SID2 :1;
    };
} RXF15SIDLbits_t;
extern volatile RXF15SIDLbits_t RXF15SIDLbits __attribute__((address(0xE6D)));
# 4944 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF15EIDH __attribute__((address(0xE6E)));

__asm("RXF15EIDH equ 0E6Eh");


typedef union {
    struct {
        unsigned int RXFEID :8;
    };
    struct {
        unsigned int EID8 :1;
        unsigned int EID9 :1;
        unsigned int EID10 :1;
        unsigned int EID11 :1;
        unsigned int EID12 :1;
        unsigned int EID13 :1;
        unsigned int EID14 :1;
        unsigned int EID15 :1;
    };
    struct {
        unsigned int RXF15EID8 :1;
        unsigned int RXF15EID9 :1;
        unsigned int RXF15EID10 :1;
        unsigned int RXF15EID11 :1;
        unsigned int RXF15EID12 :1;
        unsigned int RXF15EID13 :1;
        unsigned int RXF15EID14 :1;
        unsigned int RXF15EID15 :1;
    };
} RXF15EIDHbits_t;
extern volatile RXF15EIDHbits_t RXF15EIDHbits __attribute__((address(0xE6E)));
# 5064 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF15EIDL __attribute__((address(0xE6F)));

__asm("RXF15EIDL equ 0E6Fh");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID0 :1;
        unsigned int EID1 :1;
        unsigned int EID2 :1;
        unsigned int EID3 :1;
        unsigned int EID4 :1;
        unsigned int EID5 :1;
        unsigned int EID6 :1;
        unsigned int EID7 :1;
    };
    struct {
        unsigned int RXF15EID0 :1;
        unsigned int RXF15EID1 :1;
        unsigned int RXF15EID2 :1;
        unsigned int RXF15EID3 :1;
        unsigned int RXF15EID4 :1;
        unsigned int RXF15EID5 :1;
        unsigned int RXF15EID6 :1;
        unsigned int RXF15EID7 :1;
    };
} RXF15EIDLbits_t;
extern volatile RXF15EIDLbits_t RXF15EIDLbits __attribute__((address(0xE6F)));
# 5184 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char SDFLC __attribute__((address(0xE70)));

__asm("SDFLC equ 0E70h");


typedef union {
    struct {
        unsigned int FLC :5;
    };
    struct {
        unsigned int FLC0 :1;
        unsigned int FLC1 :1;
        unsigned int FLC2 :1;
        unsigned int FLC3 :1;
        unsigned int FLC4 :1;
    };
} SDFLCbits_t;
extern volatile SDFLCbits_t SDFLCbits __attribute__((address(0xE70)));
# 5236 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXFBCON0 __attribute__((address(0xE71)));

__asm("RXFBCON0 equ 0E71h");


typedef union {
    struct {
        unsigned int F0BP :4;
        unsigned int F1BP :4;
    };
    struct {
        unsigned int F0BP_0 :1;
        unsigned int F0BP_1 :1;
        unsigned int F0BP_2 :1;
        unsigned int F0BP_3 :1;
        unsigned int F1BP_0 :1;
        unsigned int F1BP_1 :1;
        unsigned int F1BP_2 :1;
        unsigned int F1BP_3 :1;
    };
    struct {
        unsigned int F0BP_01 :1;
    };
} RXFBCON0bits_t;
extern volatile RXFBCON0bits_t RXFBCON0bits __attribute__((address(0xE71)));
# 5320 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXFBCON1 __attribute__((address(0xE72)));

__asm("RXFBCON1 equ 0E72h");


typedef union {
    struct {
        unsigned int F2BP :4;
        unsigned int F3BP :4;
    };
    struct {
        unsigned int F2BP_0 :1;
        unsigned int F2BP_1 :1;
        unsigned int F2BP_2 :1;
        unsigned int F2BP_3 :1;
        unsigned int F3BP_0 :1;
        unsigned int F3BP_1 :1;
        unsigned int F3BP_2 :1;
        unsigned int F3BP_3 :1;
    };
    struct {
        unsigned int F2BP_01 :1;
    };
} RXFBCON1bits_t;
extern volatile RXFBCON1bits_t RXFBCON1bits __attribute__((address(0xE72)));
# 5404 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXFBCON2 __attribute__((address(0xE73)));

__asm("RXFBCON2 equ 0E73h");


typedef union {
    struct {
        unsigned int F4BP :4;
        unsigned int F5BP :4;
    };
    struct {
        unsigned int F4BP_0 :1;
        unsigned int F4BP_1 :1;
        unsigned int F4BP_2 :1;
        unsigned int F4BP_3 :1;
        unsigned int F5BP_0 :1;
        unsigned int F5BP_1 :1;
        unsigned int F5BP_2 :1;
        unsigned int F5BP_3 :1;
    };
    struct {
        unsigned int F4BP_01 :1;
    };
} RXFBCON2bits_t;
extern volatile RXFBCON2bits_t RXFBCON2bits __attribute__((address(0xE73)));
# 5488 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXFBCON3 __attribute__((address(0xE74)));

__asm("RXFBCON3 equ 0E74h");


typedef union {
    struct {
        unsigned int F6BP :4;
        unsigned int F7BP :4;
    };
    struct {
        unsigned int F6BP_0 :1;
        unsigned int F6BP_1 :1;
        unsigned int F6BP_2 :1;
        unsigned int F6BP_3 :1;
        unsigned int F7BP_0 :1;
        unsigned int F7BP_1 :1;
        unsigned int F7BP_2 :1;
        unsigned int F7BP_3 :1;
    };
    struct {
        unsigned int F6BP_01 :1;
    };
} RXFBCON3bits_t;
extern volatile RXFBCON3bits_t RXFBCON3bits __attribute__((address(0xE74)));
# 5572 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXFBCON4 __attribute__((address(0xE75)));

__asm("RXFBCON4 equ 0E75h");


typedef union {
    struct {
        unsigned int F8BP :4;
        unsigned int F9BP :4;
    };
    struct {
        unsigned int F8BP_0 :1;
        unsigned int F8BP_1 :1;
        unsigned int F8BP_2 :1;
        unsigned int F8BP_3 :1;
        unsigned int F9BP_0 :1;
        unsigned int F9BP_1 :1;
        unsigned int F9BP_2 :1;
        unsigned int F9BP_3 :1;
    };
    struct {
        unsigned int F8BP_01 :1;
    };
} RXFBCON4bits_t;
extern volatile RXFBCON4bits_t RXFBCON4bits __attribute__((address(0xE75)));
# 5656 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXFBCON5 __attribute__((address(0xE76)));

__asm("RXFBCON5 equ 0E76h");


typedef union {
    struct {
        unsigned int F10BP :4;
        unsigned int F11BP :4;
    };
    struct {
        unsigned int F10BP_0 :1;
        unsigned int F10BP_1 :1;
        unsigned int F10BP_2 :1;
        unsigned int F10BP_3 :1;
        unsigned int F11BP_0 :1;
        unsigned int F11BP_1 :1;
        unsigned int F11BP_2 :1;
        unsigned int F11BP_3 :1;
    };
    struct {
        unsigned int F10BP_01 :1;
    };
} RXFBCON5bits_t;
extern volatile RXFBCON5bits_t RXFBCON5bits __attribute__((address(0xE76)));
# 5740 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXFBCON6 __attribute__((address(0xE77)));

__asm("RXFBCON6 equ 0E77h");


typedef union {
    struct {
        unsigned int F12BP :4;
        unsigned int F13BP :4;
    };
    struct {
        unsigned int F12BP_0 :1;
        unsigned int F12BP_1 :1;
        unsigned int F12BP_2 :1;
        unsigned int F12BP_3 :1;
        unsigned int F13BP_0 :1;
        unsigned int F13BP_1 :1;
        unsigned int F13BP_2 :1;
        unsigned int F13BP_3 :1;
    };
    struct {
        unsigned int F12BP_01 :1;
    };
} RXFBCON6bits_t;
extern volatile RXFBCON6bits_t RXFBCON6bits __attribute__((address(0xE77)));
# 5824 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXFBCON7 __attribute__((address(0xE78)));

__asm("RXFBCON7 equ 0E78h");


typedef union {
    struct {
        unsigned int F14BP :4;
        unsigned int F15BP :4;
    };
    struct {
        unsigned int F14BP_0 :1;
        unsigned int F14BP_1 :1;
        unsigned int F14BP_2 :1;
        unsigned int F14BP_3 :1;
        unsigned int F15BP_0 :1;
        unsigned int F15BP_1 :1;
        unsigned int F15BP_2 :1;
        unsigned int F15BP_3 :1;
    };
    struct {
        unsigned int F14BP_01 :1;
    };
} RXFBCON7bits_t;
extern volatile RXFBCON7bits_t RXFBCON7bits __attribute__((address(0xE78)));
# 5908 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char MSEL0 __attribute__((address(0xE79)));

__asm("MSEL0 equ 0E79h");


typedef union {
    struct {
        unsigned int FIL0 :2;
        unsigned int FIL1 :2;
        unsigned int FIL2 :2;
        unsigned int FIL3 :2;
    };
    struct {
        unsigned int FIL0_0 :1;
        unsigned int FIL0_1 :1;
        unsigned int FIL1_0 :1;
        unsigned int FIL1_1 :1;
        unsigned int FIL2_0 :1;
        unsigned int FIL2_1 :1;
        unsigned int FIL3_0 :1;
        unsigned int FIL3_1 :1;
    };
} MSEL0bits_t;
extern volatile MSEL0bits_t MSEL0bits __attribute__((address(0xE79)));
# 5996 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char MSEL1 __attribute__((address(0xE7A)));

__asm("MSEL1 equ 0E7Ah");


typedef union {
    struct {
        unsigned int FIL4 :2;
        unsigned int FIL5 :2;
        unsigned int FIL6 :2;
        unsigned int FIL7 :2;
    };
    struct {
        unsigned int FIL4_0 :1;
        unsigned int FIL4_1 :1;
        unsigned int FIL5_0 :1;
        unsigned int FIL5_1 :1;
        unsigned int FIL6_0 :1;
        unsigned int FIL6_1 :1;
        unsigned int FIL7_0 :1;
        unsigned int FIL7_1 :1;
    };
} MSEL1bits_t;
extern volatile MSEL1bits_t MSEL1bits __attribute__((address(0xE7A)));
# 6084 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char MSEL2 __attribute__((address(0xE7B)));

__asm("MSEL2 equ 0E7Bh");


typedef union {
    struct {
        unsigned int FIL8 :2;
        unsigned int FIL9 :2;
        unsigned int FIL10 :2;
        unsigned int FIL11 :2;
    };
    struct {
        unsigned int FIL8_0 :1;
        unsigned int FIL8_1 :1;
        unsigned int FIL9_0 :1;
        unsigned int FIL9_1 :1;
        unsigned int FIL10_0 :1;
        unsigned int FIL10_1 :1;
        unsigned int FIL11_0 :1;
        unsigned int FIL11_1 :1;
    };
} MSEL2bits_t;
extern volatile MSEL2bits_t MSEL2bits __attribute__((address(0xE7B)));
# 6172 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char MSEL3 __attribute__((address(0xE7C)));

__asm("MSEL3 equ 0E7Ch");


typedef union {
    struct {
        unsigned int FIL12 :2;
        unsigned int FIL13 :2;
        unsigned int FIL14 :2;
        unsigned int FIL15 :2;
    };
    struct {
        unsigned int FIL12_0 :1;
        unsigned int FIL12_1 :1;
        unsigned int FIL13_0 :1;
        unsigned int FIL13_1 :1;
        unsigned int FIL14_0 :1;
        unsigned int FIL14_1 :1;
        unsigned int FIL15_0 :1;
        unsigned int FIL15_1 :1;
    };
} MSEL3bits_t;
extern volatile MSEL3bits_t MSEL3bits __attribute__((address(0xE7C)));
# 6260 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char BSEL0 __attribute__((address(0xE7D)));

__asm("BSEL0 equ 0E7Dh");


typedef union {
    struct {
        unsigned int :2;
        unsigned int B0TXEN :1;
        unsigned int B1TXEN :1;
        unsigned int B2TXEN :1;
        unsigned int B3TXEN :1;
        unsigned int B4TXEN :1;
        unsigned int B5TXEN :1;
    };
} BSEL0bits_t;
extern volatile BSEL0bits_t BSEL0bits __attribute__((address(0xE7D)));
# 6311 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char BIE0 __attribute__((address(0xE7E)));

__asm("BIE0 equ 0E7Eh");


typedef union {
    struct {
        unsigned int RXB0IE :1;
        unsigned int RXB1IE :1;
        unsigned int B0IE :1;
        unsigned int B1IE :1;
        unsigned int B2IE :1;
        unsigned int B3IE :1;
        unsigned int B4IE :1;
        unsigned int B5IE :1;
    };
    struct {
        unsigned int DRXB0IE :1;
        unsigned int DRXB1IE :1;
    };
} BIE0bits_t;
extern volatile BIE0bits_t BIE0bits __attribute__((address(0xE7E)));
# 6387 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXBIE __attribute__((address(0xE7F)));

__asm("TXBIE equ 0E7Fh");


typedef union {
    struct {
        unsigned int :2;
        unsigned int TXB0IE :1;
        unsigned int TXB1IE :1;
        unsigned int TXB2IE :1;
    };
    struct {
        unsigned int :2;
        unsigned int TX0IE :1;
        unsigned int TX1IE :1;
        unsigned int TX2IE :1;
    };
} TXBIEbits_t;
extern volatile TXBIEbits_t TXBIEbits __attribute__((address(0xE7F)));
# 6441 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B0CON __attribute__((address(0xE80)));

__asm("B0CON equ 0E80h");


typedef union {
    struct {
        unsigned int FILHIT0_TXPRI0 :1;
        unsigned int FILHIT1_TXPRI1 :1;
        unsigned int FILHIT2_RTREN :1;
        unsigned int FILHIT3_TXREQ :1;
        unsigned int FILHIT4_TXERR :1;
        unsigned int RXRTRRO_TXLARB :1;
        unsigned int RXM1_TXABT :1;
        unsigned int RXFUL_TXBIF :1;
    };
    struct {
        unsigned int FILHIT0 :1;
        unsigned int FILHIT1 :1;
        unsigned int FILHIT2 :1;
        unsigned int FILHIT3 :1;
        unsigned int FILHIT4 :1;
        unsigned int RXRTRRO :1;
        unsigned int RXM1 :1;
        unsigned int RXFUL :1;
    };
    struct {
        unsigned int TXPRI0 :1;
        unsigned int TXPRI1 :1;
        unsigned int RTREN :1;
        unsigned int TXREQ :1;
        unsigned int TXERR :1;
        unsigned int TXLARB :1;
        unsigned int TXABT :1;
        unsigned int TXBIF :1;
    };
    struct {
        unsigned int B0FILHIT0 :1;
        unsigned int B0FILHIT1 :1;
        unsigned int B0FILHIT2 :1;
        unsigned int B0FILHIT3 :1;
        unsigned int B0FILHIT4 :1;
        unsigned int B0RTRRO :1;
        unsigned int B0RXM1 :1;
        unsigned int B0RXFUL :1;
    };
    struct {
        unsigned int B0TXPRI0 :1;
        unsigned int B0TXPRI1 :1;
        unsigned int B0RTREN :1;
        unsigned int B0TXREQ :1;
        unsigned int B0TXERR :1;
        unsigned int B0TXLARB :1;
        unsigned int B0TXABT :1;
        unsigned int B0TXB3IF :1;
    };
} B0CONbits_t;
extern volatile B0CONbits_t B0CONbits __attribute__((address(0xE80)));
# 6703 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B0SIDH __attribute__((address(0xE81)));

__asm("B0SIDH equ 0E81h");


typedef union {
    struct {
        unsigned int SID :8;
    };
    struct {
        unsigned int SID3 :1;
        unsigned int SID4 :1;
        unsigned int SID5 :1;
        unsigned int SID6 :1;
        unsigned int SID7 :1;
        unsigned int SID8 :1;
        unsigned int SID9 :1;
        unsigned int SID10 :1;
    };
    struct {
        unsigned int B0SID3 :1;
        unsigned int B0SID4 :1;
        unsigned int B0SID5 :1;
        unsigned int B0SID6 :1;
        unsigned int B0SID7 :1;
        unsigned int B0SID8 :1;
        unsigned int B0SID9 :1;
        unsigned int B0SID10 :1;
    };
} B0SIDHbits_t;
extern volatile B0SIDHbits_t B0SIDHbits __attribute__((address(0xE81)));
# 6823 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B0SIDL __attribute__((address(0xE82)));

__asm("B0SIDL equ 0E82h");


typedef union {
    struct {
        unsigned int EID :2;
        unsigned int :1;
        unsigned int EXIDE :1;
        unsigned int SRR :1;
        unsigned int SID :3;
    };
    struct {
        unsigned int EID16 :1;
        unsigned int EID17 :1;
        unsigned int :3;
        unsigned int SID0 :1;
        unsigned int SID1 :1;
        unsigned int SID2 :1;
    };
    struct {
        unsigned int B0EID16 :1;
        unsigned int B0EID17 :1;
        unsigned int :1;
        unsigned int B0EXID :1;
        unsigned int B0SRR :1;
        unsigned int B0SID0 :1;
        unsigned int B0SID1 :1;
        unsigned int B0SID2 :1;
    };
} B0SIDLbits_t;
extern volatile B0SIDLbits_t B0SIDLbits __attribute__((address(0xE82)));
# 6940 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B0EIDH __attribute__((address(0xE83)));

__asm("B0EIDH equ 0E83h");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID8 :1;
        unsigned int EID9 :1;
        unsigned int EID10 :1;
        unsigned int EID11 :1;
        unsigned int EID12 :1;
        unsigned int EID13 :1;
        unsigned int EID14 :1;
        unsigned int EID15 :1;
    };
    struct {
        unsigned int B0EID8 :1;
        unsigned int B0EID9 :1;
        unsigned int B0EID10 :1;
        unsigned int B0EID11 :1;
        unsigned int B0EID12 :1;
        unsigned int B0EID13 :1;
        unsigned int B0EID14 :1;
        unsigned int B0EID15 :1;
    };
} B0EIDHbits_t;
extern volatile B0EIDHbits_t B0EIDHbits __attribute__((address(0xE83)));
# 7060 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B0EIDL __attribute__((address(0xE84)));

__asm("B0EIDL equ 0E84h");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID0 :1;
        unsigned int EID1 :1;
        unsigned int EID2 :1;
        unsigned int EID3 :1;
        unsigned int EID4 :1;
        unsigned int EID5 :1;
        unsigned int EID6 :1;
        unsigned int EID7 :1;
    };
    struct {
        unsigned int B0EID0 :1;
        unsigned int B0EID1 :1;
        unsigned int B0EID2 :1;
        unsigned int B0EID3 :1;
        unsigned int B0EID4 :1;
        unsigned int B0EID5 :1;
        unsigned int B0EID6 :1;
        unsigned int B0EID7 :1;
    };
} B0EIDLbits_t;
extern volatile B0EIDLbits_t B0EIDLbits __attribute__((address(0xE84)));
# 7180 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B0DLC __attribute__((address(0xE85)));

__asm("B0DLC equ 0E85h");


typedef union {
    struct {
        unsigned int DLC :4;
        unsigned int RB :2;
        unsigned int RXRTR_TXRTR :1;
    };
    struct {
        unsigned int DLC0 :1;
        unsigned int DLC1 :1;
        unsigned int DLC2 :1;
        unsigned int DLC3 :1;
        unsigned int RB0 :1;
        unsigned int RB1 :1;
        unsigned int RXRTR :1;
    };
    struct {
        unsigned int :6;
        unsigned int TXRTR :1;
    };
    struct {
        unsigned int B0DLC0 :1;
        unsigned int B0DLC1 :1;
        unsigned int B0DLC2 :1;
        unsigned int B0DLC3 :1;
        unsigned int B0RB0 :1;
        unsigned int B0RB1 :1;
        unsigned int B0RXRTR :1;
    };
} B0DLCbits_t;
extern volatile B0DLCbits_t B0DLCbits __attribute__((address(0xE85)));
# 7309 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B0D0 __attribute__((address(0xE86)));

__asm("B0D0 equ 0E86h");


typedef union {
    struct {
        unsigned int B0D0 :8;
    };
    struct {
        unsigned int B0D00 :1;
        unsigned int B0D01 :1;
        unsigned int B0D02 :1;
        unsigned int B0D03 :1;
        unsigned int B0D04 :1;
        unsigned int B0D05 :1;
        unsigned int B0D06 :1;
        unsigned int B0D07 :1;
    };
} B0D0bits_t;
extern volatile B0D0bits_t B0D0bits __attribute__((address(0xE86)));
# 7379 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B0D1 __attribute__((address(0xE87)));

__asm("B0D1 equ 0E87h");


typedef union {
    struct {
        unsigned int B0D1 :8;
    };
    struct {
        unsigned int B0D10 :1;
        unsigned int B0D11 :1;
        unsigned int B0D12 :1;
        unsigned int B0D13 :1;
        unsigned int B0D14 :1;
        unsigned int B0D15 :1;
        unsigned int B0D16 :1;
        unsigned int B0D17 :1;
    };
} B0D1bits_t;
extern volatile B0D1bits_t B0D1bits __attribute__((address(0xE87)));
# 7449 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B0D2 __attribute__((address(0xE88)));

__asm("B0D2 equ 0E88h");


typedef union {
    struct {
        unsigned int B0D2 :8;
    };
    struct {
        unsigned int B0D20 :1;
        unsigned int B0D21 :1;
        unsigned int B0D22 :1;
        unsigned int B0D23 :1;
        unsigned int B0D24 :1;
        unsigned int B0D25 :1;
        unsigned int B0D26 :1;
        unsigned int B0D27 :1;
    };
} B0D2bits_t;
extern volatile B0D2bits_t B0D2bits __attribute__((address(0xE88)));
# 7519 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B0D3 __attribute__((address(0xE89)));

__asm("B0D3 equ 0E89h");


typedef union {
    struct {
        unsigned int B0D3 :8;
    };
    struct {
        unsigned int B0D30 :1;
        unsigned int B0D31 :1;
        unsigned int B0D32 :1;
        unsigned int B0D33 :1;
        unsigned int B0D34 :1;
        unsigned int B0D35 :1;
        unsigned int B0D36 :1;
        unsigned int B0D37 :1;
    };
} B0D3bits_t;
extern volatile B0D3bits_t B0D3bits __attribute__((address(0xE89)));
# 7589 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B0D4 __attribute__((address(0xE8A)));

__asm("B0D4 equ 0E8Ah");


typedef union {
    struct {
        unsigned int B0D4 :8;
    };
    struct {
        unsigned int B0D40 :1;
        unsigned int B0D41 :1;
        unsigned int B0D42 :1;
        unsigned int B0D43 :1;
        unsigned int B0D44 :1;
        unsigned int B0D45 :1;
        unsigned int B0D46 :1;
        unsigned int B0D47 :1;
    };
} B0D4bits_t;
extern volatile B0D4bits_t B0D4bits __attribute__((address(0xE8A)));
# 7659 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B0D5 __attribute__((address(0xE8B)));

__asm("B0D5 equ 0E8Bh");


typedef union {
    struct {
        unsigned int B0D5 :8;
    };
    struct {
        unsigned int B0D50 :1;
        unsigned int B0D51 :1;
        unsigned int B0D52 :1;
        unsigned int B0D53 :1;
        unsigned int B0D54 :1;
        unsigned int B0D55 :1;
        unsigned int B0D56 :1;
        unsigned int B0D57 :1;
    };
} B0D5bits_t;
extern volatile B0D5bits_t B0D5bits __attribute__((address(0xE8B)));
# 7729 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B0D6 __attribute__((address(0xE8C)));

__asm("B0D6 equ 0E8Ch");


typedef union {
    struct {
        unsigned int B0D6 :8;
    };
    struct {
        unsigned int B0D60 :1;
        unsigned int B0D61 :1;
        unsigned int B0D62 :1;
        unsigned int B0D63 :1;
        unsigned int B0D64 :1;
        unsigned int B0D65 :1;
        unsigned int B0D66 :1;
        unsigned int B0D67 :1;
    };
} B0D6bits_t;
extern volatile B0D6bits_t B0D6bits __attribute__((address(0xE8C)));
# 7799 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B0D7 __attribute__((address(0xE8D)));

__asm("B0D7 equ 0E8Dh");


typedef union {
    struct {
        unsigned int B0D7 :8;
    };
    struct {
        unsigned int B0D70 :1;
        unsigned int B0D71 :1;
        unsigned int B0D72 :1;
        unsigned int B0D73 :1;
        unsigned int B0D74 :1;
        unsigned int B0D75 :1;
        unsigned int B0D76 :1;
        unsigned int B0D77 :1;
    };
} B0D7bits_t;
extern volatile B0D7bits_t B0D7bits __attribute__((address(0xE8D)));
# 7869 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CANSTAT_RO9 __attribute__((address(0xE8E)));

__asm("CANSTAT_RO9 equ 0E8Eh");


typedef union {
    struct {
        unsigned int EICODE0 :1;
        unsigned int EICODE1_ICODE0 :1;
        unsigned int EICODE2_ICODE1 :1;
        unsigned int EICODE3_ICODE2 :1;
        unsigned int EICODE4 :1;
        unsigned int OPMODE :3;
    };
    struct {
        unsigned int :1;
        unsigned int EICODE1 :1;
        unsigned int EICODE2 :1;
        unsigned int EICODE3 :1;
        unsigned int :1;
        unsigned int OPMODE0 :1;
        unsigned int OPMODE1 :1;
        unsigned int OPMODE2 :1;
    };
    struct {
        unsigned int :1;
        unsigned int ICODE0 :1;
        unsigned int ICODE1 :1;
        unsigned int ICODE2 :1;
    };
} CANSTAT_RO9bits_t;
extern volatile CANSTAT_RO9bits_t CANSTAT_RO9bits __attribute__((address(0xE8E)));
# 7980 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CANCON_RO9 __attribute__((address(0xE8F)));

__asm("CANCON_RO9 equ 0E8Fh");


typedef union {
    struct {
        unsigned int FP0 :1;
        unsigned int WIN0_FP1 :1;
        unsigned int WIN1_FP2 :1;
        unsigned int WIN2_FP3 :1;
        unsigned int ABAT :1;
        unsigned int REQOP :3;
    };
    struct {
        unsigned int :1;
        unsigned int WIN0 :1;
        unsigned int WIN1 :1;
        unsigned int WIN2 :1;
    };
    struct {
        unsigned int :1;
        unsigned int FP1 :1;
        unsigned int FP2 :1;
        unsigned int FP3 :1;
    };
} CANCON_RO9bits_t;
extern volatile CANCON_RO9bits_t CANCON_RO9bits __attribute__((address(0xE8F)));
# 8072 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B1CON __attribute__((address(0xE90)));

__asm("B1CON equ 0E90h");


typedef union {
    struct {
        unsigned int FILHIT0_TXPRI0 :1;
        unsigned int FILHIT1_TXPRI1 :1;
        unsigned int FILHIT2_RTREN :1;
        unsigned int FILHIT3_TXREQ :1;
        unsigned int FILHIT4_TXERR :1;
        unsigned int RXRTRRO_TXLARB :1;
        unsigned int RXM1_TXABT :1;
        unsigned int RXFUL_TXBIF :1;
    };
    struct {
        unsigned int FILHIT0 :1;
        unsigned int FILHIT1 :1;
        unsigned int FILHIT2 :1;
        unsigned int FILHIT3 :1;
        unsigned int FILHIT4 :1;
        unsigned int RXRTRRO :1;
        unsigned int RXM1 :1;
        unsigned int RXFUL :1;
    };
    struct {
        unsigned int TXPRI0 :1;
        unsigned int TXPRI1 :1;
        unsigned int RTREN :1;
        unsigned int TXREQ :1;
        unsigned int TXERR :1;
        unsigned int TXLARB :1;
        unsigned int TXABT :1;
        unsigned int TXBIF :1;
    };
    struct {
        unsigned int B1FILHIT0 :1;
        unsigned int B1FILHIT1 :1;
        unsigned int B1FILHIT2 :1;
        unsigned int B1FILHIT3 :1;
        unsigned int B1FILHIT4 :1;
        unsigned int B1RTRRO :1;
        unsigned int B1RXM1 :1;
        unsigned int B1RXFUL :1;
    };
    struct {
        unsigned int B1TXPRI0 :1;
        unsigned int B1TXPRI1 :1;
        unsigned int B1RTREN :1;
        unsigned int B1TXREQ :1;
        unsigned int B1TXERR :1;
        unsigned int B1TXLARB :1;
        unsigned int B1TXABT :1;
        unsigned int B1TXB3IF :1;
    };
} B1CONbits_t;
extern volatile B1CONbits_t B1CONbits __attribute__((address(0xE90)));
# 8334 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B1SIDH __attribute__((address(0xE91)));

__asm("B1SIDH equ 0E91h");


typedef union {
    struct {
        unsigned int SID :8;
    };
    struct {
        unsigned int SID3 :1;
        unsigned int SID4 :1;
        unsigned int SID5 :1;
        unsigned int SID6 :1;
        unsigned int SID7 :1;
        unsigned int SID8 :1;
        unsigned int SID9 :1;
        unsigned int SID10 :1;
    };
    struct {
        unsigned int B1SID3 :1;
        unsigned int B1SID4 :1;
        unsigned int B1SID5 :1;
        unsigned int B1SID6 :1;
        unsigned int B1SID7 :1;
        unsigned int B1SID8 :1;
        unsigned int B1SID9 :1;
        unsigned int B1SID10 :1;
    };
} B1SIDHbits_t;
extern volatile B1SIDHbits_t B1SIDHbits __attribute__((address(0xE91)));
# 8454 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B1SIDL __attribute__((address(0xE92)));

__asm("B1SIDL equ 0E92h");


typedef union {
    struct {
        unsigned int EID :2;
        unsigned int :1;
        unsigned int EXIDE :1;
        unsigned int SRR :1;
        unsigned int SID :3;
    };
    struct {
        unsigned int EID16 :1;
        unsigned int EID17 :1;
        unsigned int :3;
        unsigned int SID0 :1;
        unsigned int SID1 :1;
        unsigned int SID2 :1;
    };
    struct {
        unsigned int B1EID16 :1;
        unsigned int B1EID17 :1;
        unsigned int :1;
        unsigned int B1EXID :1;
        unsigned int B1SRR :1;
        unsigned int B1SID0 :1;
        unsigned int B1SID1 :1;
        unsigned int B1SID2 :1;
    };
} B1SIDLbits_t;
extern volatile B1SIDLbits_t B1SIDLbits __attribute__((address(0xE92)));
# 8571 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B1EIDH __attribute__((address(0xE93)));

__asm("B1EIDH equ 0E93h");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID8 :1;
        unsigned int EID9 :1;
        unsigned int EID10 :1;
        unsigned int EID11 :1;
        unsigned int EID12 :1;
        unsigned int EID13 :1;
        unsigned int EID14 :1;
        unsigned int EID15 :1;
    };
    struct {
        unsigned int B1EID8 :1;
        unsigned int B1EID9 :1;
        unsigned int B1EID10 :1;
        unsigned int B1EID11 :1;
        unsigned int B1EID12 :1;
        unsigned int B1EID13 :1;
        unsigned int B1EID14 :1;
        unsigned int B1EID15 :1;
    };
} B1EIDHbits_t;
extern volatile B1EIDHbits_t B1EIDHbits __attribute__((address(0xE93)));
# 8691 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B1EIDL __attribute__((address(0xE94)));

__asm("B1EIDL equ 0E94h");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID0 :1;
        unsigned int EID1 :1;
        unsigned int EID2 :1;
        unsigned int EID3 :1;
        unsigned int EID4 :1;
        unsigned int EID5 :1;
        unsigned int EID6 :1;
        unsigned int EID7 :1;
    };
    struct {
        unsigned int B1EID0 :1;
        unsigned int B1EID1 :1;
        unsigned int B1EID2 :1;
        unsigned int B1EID3 :1;
        unsigned int B1EID4 :1;
        unsigned int B1EID5 :1;
        unsigned int B1EID6 :1;
        unsigned int B1EID7 :1;
    };
} B1EIDLbits_t;
extern volatile B1EIDLbits_t B1EIDLbits __attribute__((address(0xE94)));
# 8811 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B1DLC __attribute__((address(0xE95)));

__asm("B1DLC equ 0E95h");


typedef union {
    struct {
        unsigned int DLC :4;
        unsigned int RB :2;
        unsigned int RXRTR_TXRTR :1;
    };
    struct {
        unsigned int DLC0 :1;
        unsigned int DLC1 :1;
        unsigned int DLC2 :1;
        unsigned int DLC3 :1;
        unsigned int RB0 :1;
        unsigned int RB1 :1;
        unsigned int RXRTR :1;
    };
    struct {
        unsigned int :6;
        unsigned int TXRTR :1;
    };
    struct {
        unsigned int B1DLC0 :1;
        unsigned int B1DLC1 :1;
        unsigned int B1DLC2 :1;
        unsigned int B1DLC3 :1;
        unsigned int B1RB0 :1;
        unsigned int B1RB1 :1;
        unsigned int B1RXRTR :1;
    };
} B1DLCbits_t;
extern volatile B1DLCbits_t B1DLCbits __attribute__((address(0xE95)));
# 8940 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B1D0 __attribute__((address(0xE96)));

__asm("B1D0 equ 0E96h");


typedef union {
    struct {
        unsigned int B1D0 :8;
    };
    struct {
        unsigned int B1D00 :1;
        unsigned int B1D01 :1;
        unsigned int B1D02 :1;
        unsigned int B1D03 :1;
        unsigned int B1D04 :1;
        unsigned int B1D05 :1;
        unsigned int B1D06 :1;
        unsigned int B1D07 :1;
    };
} B1D0bits_t;
extern volatile B1D0bits_t B1D0bits __attribute__((address(0xE96)));
# 9010 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B1D1 __attribute__((address(0xE97)));

__asm("B1D1 equ 0E97h");


typedef union {
    struct {
        unsigned int B1D1 :8;
    };
    struct {
        unsigned int B1D10 :1;
        unsigned int B1D11 :1;
        unsigned int B1D12 :1;
        unsigned int B1D13 :1;
        unsigned int B1D14 :1;
        unsigned int B1D15 :1;
        unsigned int B1D16 :1;
        unsigned int B1D17 :1;
    };
} B1D1bits_t;
extern volatile B1D1bits_t B1D1bits __attribute__((address(0xE97)));
# 9080 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B1D2 __attribute__((address(0xE98)));

__asm("B1D2 equ 0E98h");


typedef union {
    struct {
        unsigned int B1D2 :8;
    };
    struct {
        unsigned int B1D20 :1;
        unsigned int B1D21 :1;
        unsigned int B1D22 :1;
        unsigned int B1D23 :1;
        unsigned int B1D24 :1;
        unsigned int B1D25 :1;
        unsigned int B1D26 :1;
        unsigned int B1D27 :1;
    };
} B1D2bits_t;
extern volatile B1D2bits_t B1D2bits __attribute__((address(0xE98)));
# 9150 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B1D3 __attribute__((address(0xE99)));

__asm("B1D3 equ 0E99h");


typedef union {
    struct {
        unsigned int B1D3 :8;
    };
    struct {
        unsigned int B1D30 :1;
        unsigned int B1D31 :1;
        unsigned int B1D32 :1;
        unsigned int B1D33 :1;
        unsigned int B1D34 :1;
        unsigned int B1D35 :1;
        unsigned int B1D36 :1;
        unsigned int B1D37 :1;
    };
} B1D3bits_t;
extern volatile B1D3bits_t B1D3bits __attribute__((address(0xE99)));
# 9220 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B1D4 __attribute__((address(0xE9A)));

__asm("B1D4 equ 0E9Ah");


typedef union {
    struct {
        unsigned int B1D4 :8;
    };
    struct {
        unsigned int B1D40 :1;
        unsigned int B1D41 :1;
        unsigned int B1D42 :1;
        unsigned int B1D43 :1;
        unsigned int B1D44 :1;
        unsigned int B1D45 :1;
        unsigned int B1D46 :1;
        unsigned int B1D47 :1;
    };
} B1D4bits_t;
extern volatile B1D4bits_t B1D4bits __attribute__((address(0xE9A)));
# 9290 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B1D5 __attribute__((address(0xE9B)));

__asm("B1D5 equ 0E9Bh");


typedef union {
    struct {
        unsigned int B1D5 :8;
    };
    struct {
        unsigned int B1D50 :1;
        unsigned int B1D51 :1;
        unsigned int B1D52 :1;
        unsigned int B1D53 :1;
        unsigned int B1D54 :1;
        unsigned int B1D55 :1;
        unsigned int B1D56 :1;
        unsigned int B1D57 :1;
    };
} B1D5bits_t;
extern volatile B1D5bits_t B1D5bits __attribute__((address(0xE9B)));
# 9360 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B1D6 __attribute__((address(0xE9C)));

__asm("B1D6 equ 0E9Ch");


typedef union {
    struct {
        unsigned int B1D6 :8;
    };
    struct {
        unsigned int B1D60 :1;
        unsigned int B1D61 :1;
        unsigned int B1D62 :1;
        unsigned int B1D63 :1;
        unsigned int B1D64 :1;
        unsigned int B1D65 :1;
        unsigned int B1D66 :1;
        unsigned int B1D67 :1;
    };
} B1D6bits_t;
extern volatile B1D6bits_t B1D6bits __attribute__((address(0xE9C)));
# 9430 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B1D7 __attribute__((address(0xE9D)));

__asm("B1D7 equ 0E9Dh");


typedef union {
    struct {
        unsigned int B1D7 :8;
    };
    struct {
        unsigned int B1D70 :1;
        unsigned int B1D71 :1;
        unsigned int B1D72 :1;
        unsigned int B1D73 :1;
        unsigned int B1D74 :1;
        unsigned int B1D75 :1;
        unsigned int B1D76 :1;
        unsigned int B1D77 :1;
    };
} B1D7bits_t;
extern volatile B1D7bits_t B1D7bits __attribute__((address(0xE9D)));
# 9500 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CANSTAT_RO8 __attribute__((address(0xE9E)));

__asm("CANSTAT_RO8 equ 0E9Eh");


typedef union {
    struct {
        unsigned int EICODE0 :1;
        unsigned int EICODE1_ICODE0 :1;
        unsigned int EICODE2_ICODE1 :1;
        unsigned int EICODE3_ICODE2 :1;
        unsigned int EICODE4 :1;
        unsigned int OPMODE :3;
    };
    struct {
        unsigned int :1;
        unsigned int EICODE1 :1;
        unsigned int EICODE2 :1;
        unsigned int EICODE3 :1;
        unsigned int :1;
        unsigned int OPMODE0 :1;
        unsigned int OPMODE1 :1;
        unsigned int OPMODE2 :1;
    };
    struct {
        unsigned int :1;
        unsigned int ICODE0 :1;
        unsigned int ICODE1 :1;
        unsigned int ICODE2 :1;
    };
} CANSTAT_RO8bits_t;
extern volatile CANSTAT_RO8bits_t CANSTAT_RO8bits __attribute__((address(0xE9E)));
# 9611 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CANCON_RO8 __attribute__((address(0xE9F)));

__asm("CANCON_RO8 equ 0E9Fh");


typedef union {
    struct {
        unsigned int FP0 :1;
        unsigned int WIN0_FP1 :1;
        unsigned int WIN1_FP2 :1;
        unsigned int WIN2_FP3 :1;
        unsigned int ABAT :1;
        unsigned int REQOP :3;
    };
    struct {
        unsigned int :1;
        unsigned int WIN0 :1;
        unsigned int WIN1 :1;
        unsigned int WIN2 :1;
    };
    struct {
        unsigned int :1;
        unsigned int FP1 :1;
        unsigned int FP2 :1;
        unsigned int FP3 :1;
    };
} CANCON_RO8bits_t;
extern volatile CANCON_RO8bits_t CANCON_RO8bits __attribute__((address(0xE9F)));
# 9703 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B2CON __attribute__((address(0xEA0)));

__asm("B2CON equ 0EA0h");


typedef union {
    struct {
        unsigned int FILHIT0_TXPRI0 :1;
        unsigned int FILHIT1_TXPRI1 :1;
        unsigned int FILHIT2_RTREN :1;
        unsigned int FILHIT3_TXREQ :1;
        unsigned int FILHIT4_TXERR :1;
        unsigned int RXRTRRO_TXLARB :1;
        unsigned int RXM1_TXABT :1;
        unsigned int RXFUL_TXBIF :1;
    };
    struct {
        unsigned int FILHIT0 :1;
        unsigned int FILHIT1 :1;
        unsigned int FILHIT2 :1;
        unsigned int FILHIT3 :1;
        unsigned int FILHIT4 :1;
        unsigned int RXRTRRO :1;
        unsigned int RXM1 :1;
        unsigned int RXFUL :1;
    };
    struct {
        unsigned int TXPRI0 :1;
        unsigned int TXPRI1 :1;
        unsigned int RTREN :1;
        unsigned int TXREQ :1;
        unsigned int TXERR :1;
        unsigned int TXLARB :1;
        unsigned int TXABT :1;
        unsigned int TXBIF :1;
    };
    struct {
        unsigned int B2FILHIT0 :1;
        unsigned int B2FILHIT1 :1;
        unsigned int B2FILHIT2 :1;
        unsigned int B2FILHIT3 :1;
        unsigned int B2FILHIT4 :1;
        unsigned int B2RTRRO :1;
        unsigned int B2RXM1 :1;
        unsigned int B2RXFUL :1;
    };
    struct {
        unsigned int B2TXPRI0 :1;
        unsigned int B2TXPRI1 :1;
        unsigned int B2RTREN :1;
        unsigned int B2TXREQ :1;
        unsigned int B2TXERR :1;
        unsigned int B2TXLARB :1;
        unsigned int B2TXABT :1;
        unsigned int B2TXB3IF :1;
    };
} B2CONbits_t;
extern volatile B2CONbits_t B2CONbits __attribute__((address(0xEA0)));
# 9965 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B2SIDH __attribute__((address(0xEA1)));

__asm("B2SIDH equ 0EA1h");


typedef union {
    struct {
        unsigned int SID :8;
    };
    struct {
        unsigned int SID3 :1;
        unsigned int SID4 :1;
        unsigned int SID5 :1;
        unsigned int SID6 :1;
        unsigned int SID7 :1;
        unsigned int SID8 :1;
        unsigned int SID9 :1;
        unsigned int SID10 :1;
    };
    struct {
        unsigned int B2SID3 :1;
        unsigned int B2SID4 :1;
        unsigned int B2SID5 :1;
        unsigned int B2SID6 :1;
        unsigned int B2SID7 :1;
        unsigned int B2SID8 :1;
        unsigned int B2SID9 :1;
        unsigned int B2SID10 :1;
    };
} B2SIDHbits_t;
extern volatile B2SIDHbits_t B2SIDHbits __attribute__((address(0xEA1)));
# 10085 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B2SIDL __attribute__((address(0xEA2)));

__asm("B2SIDL equ 0EA2h");


typedef union {
    struct {
        unsigned int EID :2;
        unsigned int :1;
        unsigned int EXIDE :1;
        unsigned int SRR :1;
        unsigned int SID :3;
    };
    struct {
        unsigned int EID16 :1;
        unsigned int EID17 :1;
        unsigned int :3;
        unsigned int SID0 :1;
        unsigned int SID1 :1;
        unsigned int SID2 :1;
    };
    struct {
        unsigned int B2EID16 :1;
        unsigned int B2EID17 :1;
        unsigned int :1;
        unsigned int B2EXID :1;
        unsigned int B2SRR :1;
        unsigned int B2SID0 :1;
        unsigned int B2SID1 :1;
        unsigned int B2SID2 :1;
    };
    struct {
        unsigned int :3;
        unsigned int B2EXIDE :1;
    };
} B2SIDLbits_t;
extern volatile B2SIDLbits_t B2SIDLbits __attribute__((address(0xEA2)));
# 10211 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B2EIDH __attribute__((address(0xEA3)));

__asm("B2EIDH equ 0EA3h");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID8 :1;
        unsigned int EID9 :1;
        unsigned int EID10 :1;
        unsigned int EID11 :1;
        unsigned int EID12 :1;
        unsigned int EID13 :1;
        unsigned int EID14 :1;
        unsigned int EID15 :1;
    };
    struct {
        unsigned int B2EID8 :1;
        unsigned int B2EID9 :1;
        unsigned int B2EID10 :1;
        unsigned int B2EID11 :1;
        unsigned int B2EID12 :1;
        unsigned int B2EID13 :1;
        unsigned int B2EID14 :1;
        unsigned int B2EID15 :1;
    };
} B2EIDHbits_t;
extern volatile B2EIDHbits_t B2EIDHbits __attribute__((address(0xEA3)));
# 10331 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B2EIDL __attribute__((address(0xEA4)));

__asm("B2EIDL equ 0EA4h");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID0 :1;
        unsigned int EID1 :1;
        unsigned int EID2 :1;
        unsigned int EID3 :1;
        unsigned int EID4 :1;
        unsigned int EID5 :1;
        unsigned int EID6 :1;
        unsigned int EID7 :1;
    };
    struct {
        unsigned int B2EID0 :1;
        unsigned int B2EID1 :1;
        unsigned int B2EID2 :1;
        unsigned int B2EID3 :1;
        unsigned int B2EID4 :1;
        unsigned int B2EID5 :1;
        unsigned int B2EID6 :1;
        unsigned int B2EID7 :1;
    };
} B2EIDLbits_t;
extern volatile B2EIDLbits_t B2EIDLbits __attribute__((address(0xEA4)));
# 10451 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B2DLC __attribute__((address(0xEA5)));

__asm("B2DLC equ 0EA5h");


typedef union {
    struct {
        unsigned int DLC :4;
        unsigned int RB :2;
        unsigned int RXRTR_TXRTR :1;
    };
    struct {
        unsigned int DLC0 :1;
        unsigned int DLC1 :1;
        unsigned int DLC2 :1;
        unsigned int DLC3 :1;
        unsigned int RB0 :1;
        unsigned int RB1 :1;
        unsigned int RXRTR :1;
    };
    struct {
        unsigned int :6;
        unsigned int TXRTR :1;
    };
    struct {
        unsigned int B2DLC0 :1;
        unsigned int B2DLC1 :1;
        unsigned int B2DLC2 :1;
        unsigned int B2DLC3 :1;
        unsigned int B2RB0 :1;
        unsigned int B2RB1 :1;
        unsigned int B2RXRTR :1;
    };
} B2DLCbits_t;
extern volatile B2DLCbits_t B2DLCbits __attribute__((address(0xEA5)));
# 10580 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B2D0 __attribute__((address(0xEA6)));

__asm("B2D0 equ 0EA6h");


typedef union {
    struct {
        unsigned int B2D0 :8;
    };
    struct {
        unsigned int B2D00 :1;
        unsigned int B2D01 :1;
        unsigned int B2D02 :1;
        unsigned int B2D03 :1;
        unsigned int B2D04 :1;
        unsigned int B2D05 :1;
        unsigned int B2D06 :1;
        unsigned int B2D07 :1;
    };
} B2D0bits_t;
extern volatile B2D0bits_t B2D0bits __attribute__((address(0xEA6)));
# 10650 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B2D1 __attribute__((address(0xEA7)));

__asm("B2D1 equ 0EA7h");


typedef union {
    struct {
        unsigned int B2D1 :8;
    };
    struct {
        unsigned int B2D10 :1;
        unsigned int B2D11 :1;
        unsigned int B2D12 :1;
        unsigned int B2D13 :1;
        unsigned int B2D14 :1;
        unsigned int B2D15 :1;
        unsigned int B2D16 :1;
        unsigned int B2D17 :1;
    };
} B2D1bits_t;
extern volatile B2D1bits_t B2D1bits __attribute__((address(0xEA7)));
# 10720 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B2D2 __attribute__((address(0xEA8)));

__asm("B2D2 equ 0EA8h");


typedef union {
    struct {
        unsigned int B2D2 :8;
    };
    struct {
        unsigned int B2D20 :1;
        unsigned int B2D21 :1;
        unsigned int B2D22 :1;
        unsigned int B2D23 :1;
        unsigned int B2D24 :1;
        unsigned int B2D25 :1;
        unsigned int B2D26 :1;
        unsigned int B2D27 :1;
    };
} B2D2bits_t;
extern volatile B2D2bits_t B2D2bits __attribute__((address(0xEA8)));
# 10790 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B2D3 __attribute__((address(0xEA9)));

__asm("B2D3 equ 0EA9h");


typedef union {
    struct {
        unsigned int B2D3 :8;
    };
    struct {
        unsigned int B2D30 :1;
        unsigned int B2D31 :1;
        unsigned int B2D32 :1;
        unsigned int B2D33 :1;
        unsigned int B2D34 :1;
        unsigned int B2D35 :1;
        unsigned int B2D36 :1;
        unsigned int B2D37 :1;
    };
} B2D3bits_t;
extern volatile B2D3bits_t B2D3bits __attribute__((address(0xEA9)));
# 10860 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B2D4 __attribute__((address(0xEAA)));

__asm("B2D4 equ 0EAAh");


typedef union {
    struct {
        unsigned int B2D4 :8;
    };
    struct {
        unsigned int B2D40 :1;
        unsigned int B2D41 :1;
        unsigned int B2D42 :1;
        unsigned int B2D43 :1;
        unsigned int B2D44 :1;
        unsigned int B2D45 :1;
        unsigned int B2D46 :1;
        unsigned int B2D47 :1;
    };
} B2D4bits_t;
extern volatile B2D4bits_t B2D4bits __attribute__((address(0xEAA)));
# 10930 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B2D5 __attribute__((address(0xEAB)));

__asm("B2D5 equ 0EABh");


typedef union {
    struct {
        unsigned int B2D5 :8;
    };
    struct {
        unsigned int B2D50 :1;
        unsigned int B2D51 :1;
        unsigned int B2D52 :1;
        unsigned int B2D53 :1;
        unsigned int B2D54 :1;
        unsigned int B2D55 :1;
        unsigned int B2D56 :1;
        unsigned int B2D57 :1;
    };
} B2D5bits_t;
extern volatile B2D5bits_t B2D5bits __attribute__((address(0xEAB)));
# 11000 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B2D6 __attribute__((address(0xEAC)));

__asm("B2D6 equ 0EACh");


typedef union {
    struct {
        unsigned int B2D6 :8;
    };
    struct {
        unsigned int B2D60 :1;
        unsigned int B2D61 :1;
        unsigned int B2D62 :1;
        unsigned int B2D63 :1;
        unsigned int B2D64 :1;
        unsigned int B2D65 :1;
        unsigned int B2D66 :1;
        unsigned int B2D67 :1;
    };
} B2D6bits_t;
extern volatile B2D6bits_t B2D6bits __attribute__((address(0xEAC)));
# 11070 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B2D7 __attribute__((address(0xEAD)));

__asm("B2D7 equ 0EADh");


typedef union {
    struct {
        unsigned int B2D7 :8;
    };
    struct {
        unsigned int B2D70 :1;
        unsigned int B2D71 :1;
        unsigned int B2D72 :1;
        unsigned int B2D73 :1;
        unsigned int B2D74 :1;
        unsigned int B2D75 :1;
        unsigned int B2D76 :1;
        unsigned int B2D77 :1;
    };
} B2D7bits_t;
extern volatile B2D7bits_t B2D7bits __attribute__((address(0xEAD)));
# 11140 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CANSTAT_RO7 __attribute__((address(0xEAE)));

__asm("CANSTAT_RO7 equ 0EAEh");


typedef union {
    struct {
        unsigned int EICODE0 :1;
        unsigned int EICODE1_ICODE0 :1;
        unsigned int EICODE2_ICODE1 :1;
        unsigned int EICODE3_ICODE2 :1;
        unsigned int EICODE4 :1;
        unsigned int OPMODE :3;
    };
    struct {
        unsigned int :1;
        unsigned int EICODE1 :1;
        unsigned int EICODE2 :1;
        unsigned int EICODE3 :1;
        unsigned int :1;
        unsigned int OPMODE0 :1;
        unsigned int OPMODE1 :1;
        unsigned int OPMODE2 :1;
    };
    struct {
        unsigned int :1;
        unsigned int ICODE0 :1;
        unsigned int ICODE1 :1;
        unsigned int ICODE2 :1;
    };
} CANSTAT_RO7bits_t;
extern volatile CANSTAT_RO7bits_t CANSTAT_RO7bits __attribute__((address(0xEAE)));
# 11251 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CANCON_RO7 __attribute__((address(0xEAF)));

__asm("CANCON_RO7 equ 0EAFh");


typedef union {
    struct {
        unsigned int FP0 :1;
        unsigned int WIN0_FP1 :1;
        unsigned int WIN1_FP2 :1;
        unsigned int WIN2_FP3 :1;
        unsigned int ABAT :1;
        unsigned int REQOP :3;
    };
    struct {
        unsigned int :1;
        unsigned int WIN0 :1;
        unsigned int WIN1 :1;
        unsigned int WIN2 :1;
    };
    struct {
        unsigned int :1;
        unsigned int FP1 :1;
        unsigned int FP2 :1;
        unsigned int FP3 :1;
    };
} CANCON_RO7bits_t;
extern volatile CANCON_RO7bits_t CANCON_RO7bits __attribute__((address(0xEAF)));
# 11343 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B3CON __attribute__((address(0xEB0)));

__asm("B3CON equ 0EB0h");


typedef union {
    struct {
        unsigned int FILHIT0_TXPRI0 :1;
        unsigned int FILHIT1_TXPRI1 :1;
        unsigned int FILHIT2_RTREN :1;
        unsigned int FILHIT3_TXREQ :1;
        unsigned int FILHIT4_TXERR :1;
        unsigned int RXRTRRO_TXLARB :1;
        unsigned int RXM1_TXABT :1;
        unsigned int RXFUL_TXBIF :1;
    };
    struct {
        unsigned int FILHIT0 :1;
        unsigned int FILHIT1 :1;
        unsigned int FILHIT2 :1;
        unsigned int FILHIT3 :1;
        unsigned int FILHIT4 :1;
        unsigned int RXRTRRO :1;
        unsigned int RXM1 :1;
        unsigned int RXFUL :1;
    };
    struct {
        unsigned int TXPRI0 :1;
        unsigned int TXPRI1 :1;
        unsigned int RTREN :1;
        unsigned int TXREQ :1;
        unsigned int TXERR :1;
        unsigned int TXLARB :1;
        unsigned int TXABT :1;
        unsigned int TXBIF :1;
    };
    struct {
        unsigned int B3FILHIT0 :1;
        unsigned int B3FILHIT1 :1;
        unsigned int B3FILHIT2 :1;
        unsigned int B3FILHIT3 :1;
        unsigned int B3FILHIT4 :1;
        unsigned int B3RTRRO :1;
        unsigned int B3RXM1 :1;
        unsigned int B3RXFUL :1;
    };
    struct {
        unsigned int B3TXPRI0 :1;
        unsigned int B3TXPRI1 :1;
        unsigned int B3RTREN :1;
        unsigned int B3TXREQ :1;
        unsigned int B3TXERR :1;
        unsigned int B3TXLARB :1;
        unsigned int B3TXABT :1;
        unsigned int B3TXB3IF :1;
    };
} B3CONbits_t;
extern volatile B3CONbits_t B3CONbits __attribute__((address(0xEB0)));
# 11605 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B3SIDH __attribute__((address(0xEB1)));

__asm("B3SIDH equ 0EB1h");


typedef union {
    struct {
        unsigned int SID :8;
    };
    struct {
        unsigned int SID3 :1;
        unsigned int SID4 :1;
        unsigned int SID5 :1;
        unsigned int SID6 :1;
        unsigned int SID7 :1;
        unsigned int SID8 :1;
        unsigned int SID9 :1;
        unsigned int SID10 :1;
    };
    struct {
        unsigned int B3SID3 :1;
        unsigned int B3SID4 :1;
        unsigned int B3SID5 :1;
        unsigned int B3SID6 :1;
        unsigned int B3SID7 :1;
        unsigned int B3SID8 :1;
        unsigned int B3SID9 :1;
        unsigned int B3SID10 :1;
    };
} B3SIDHbits_t;
extern volatile B3SIDHbits_t B3SIDHbits __attribute__((address(0xEB1)));
# 11725 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B3SIDL __attribute__((address(0xEB2)));

__asm("B3SIDL equ 0EB2h");


typedef union {
    struct {
        unsigned int EID :2;
        unsigned int :1;
        unsigned int EXIDE :1;
        unsigned int SRR :1;
        unsigned int SID :3;
    };
    struct {
        unsigned int EID16 :1;
        unsigned int EID17 :1;
        unsigned int :3;
        unsigned int SID0 :1;
        unsigned int SID1 :1;
        unsigned int SID2 :1;
    };
    struct {
        unsigned int B3EID16 :1;
        unsigned int B3EID17 :1;
        unsigned int :1;
        unsigned int B3EXID :1;
        unsigned int B3SRR :1;
        unsigned int B3SID0 :1;
        unsigned int B3SID1 :1;
        unsigned int B3SID2 :1;
    };
    struct {
        unsigned int :3;
        unsigned int B3EXIDE :1;
    };
} B3SIDLbits_t;
extern volatile B3SIDLbits_t B3SIDLbits __attribute__((address(0xEB2)));
# 11851 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B3EIDH __attribute__((address(0xEB3)));

__asm("B3EIDH equ 0EB3h");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID8 :1;
        unsigned int EID9 :1;
        unsigned int EID10 :1;
        unsigned int EID11 :1;
        unsigned int EID12 :1;
        unsigned int EID13 :1;
        unsigned int EID14 :1;
        unsigned int EID15 :1;
    };
    struct {
        unsigned int B3EID8 :1;
        unsigned int B3EID9 :1;
        unsigned int B3EID10 :1;
        unsigned int B3EID11 :1;
        unsigned int B3EID12 :1;
        unsigned int B3EID13 :1;
        unsigned int B3EID14 :1;
        unsigned int B3EID15 :1;
    };
} B3EIDHbits_t;
extern volatile B3EIDHbits_t B3EIDHbits __attribute__((address(0xEB3)));
# 11971 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B3EIDL __attribute__((address(0xEB4)));

__asm("B3EIDL equ 0EB4h");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID0 :1;
        unsigned int EID1 :1;
        unsigned int EID2 :1;
        unsigned int EID3 :1;
        unsigned int EID4 :1;
        unsigned int EID5 :1;
        unsigned int EID6 :1;
        unsigned int EID7 :1;
    };
    struct {
        unsigned int B3EID0 :1;
        unsigned int B3EID1 :1;
        unsigned int B3EID2 :1;
        unsigned int B3EID3 :1;
        unsigned int B3EID4 :1;
        unsigned int B3EID5 :1;
        unsigned int B3EID6 :1;
        unsigned int B3EID7 :1;
    };
} B3EIDLbits_t;
extern volatile B3EIDLbits_t B3EIDLbits __attribute__((address(0xEB4)));
# 12091 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B3DLC __attribute__((address(0xEB5)));

__asm("B3DLC equ 0EB5h");


typedef union {
    struct {
        unsigned int DLC :4;
        unsigned int RB :2;
        unsigned int RXRTR_TXRTR :1;
    };
    struct {
        unsigned int DLC0 :1;
        unsigned int DLC1 :1;
        unsigned int DLC2 :1;
        unsigned int DLC3 :1;
        unsigned int RB0 :1;
        unsigned int RB1 :1;
        unsigned int RXRTR :1;
    };
    struct {
        unsigned int :6;
        unsigned int TXRTR :1;
    };
    struct {
        unsigned int B3DLC0 :1;
        unsigned int B3DLC1 :1;
        unsigned int B3DLC2 :1;
        unsigned int B3DLC3 :1;
        unsigned int B3RB0 :1;
        unsigned int B3RB1 :1;
        unsigned int B3RXRTR :1;
    };
} B3DLCbits_t;
extern volatile B3DLCbits_t B3DLCbits __attribute__((address(0xEB5)));
# 12220 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B3D0 __attribute__((address(0xEB6)));

__asm("B3D0 equ 0EB6h");


typedef union {
    struct {
        unsigned int B3D0 :8;
    };
    struct {
        unsigned int B3D00 :1;
        unsigned int B3D01 :1;
        unsigned int B3D02 :1;
        unsigned int B3D03 :1;
        unsigned int B3D04 :1;
        unsigned int B3D05 :1;
        unsigned int B3D06 :1;
        unsigned int B3D07 :1;
    };
} B3D0bits_t;
extern volatile B3D0bits_t B3D0bits __attribute__((address(0xEB6)));
# 12290 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B3D1 __attribute__((address(0xEB7)));

__asm("B3D1 equ 0EB7h");


typedef union {
    struct {
        unsigned int B3D1 :8;
    };
    struct {
        unsigned int B3D10 :1;
        unsigned int B3D11 :1;
        unsigned int B3D12 :1;
        unsigned int B3D13 :1;
        unsigned int B3D14 :1;
        unsigned int B3D15 :1;
        unsigned int B3D16 :1;
        unsigned int B3D17 :1;
    };
} B3D1bits_t;
extern volatile B3D1bits_t B3D1bits __attribute__((address(0xEB7)));
# 12360 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B3D2 __attribute__((address(0xEB8)));

__asm("B3D2 equ 0EB8h");


typedef union {
    struct {
        unsigned int B3D2 :8;
    };
    struct {
        unsigned int B3D20 :1;
        unsigned int B3D21 :1;
        unsigned int B3D22 :1;
        unsigned int B3D23 :1;
        unsigned int B3D24 :1;
        unsigned int B3D25 :1;
        unsigned int B3D26 :1;
        unsigned int B3D27 :1;
    };
} B3D2bits_t;
extern volatile B3D2bits_t B3D2bits __attribute__((address(0xEB8)));
# 12430 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B3D3 __attribute__((address(0xEB9)));

__asm("B3D3 equ 0EB9h");


typedef union {
    struct {
        unsigned int B3D3 :8;
    };
    struct {
        unsigned int B3D30 :1;
        unsigned int B3D31 :1;
        unsigned int B3D32 :1;
        unsigned int B3D33 :1;
        unsigned int B3D34 :1;
        unsigned int B3D35 :1;
        unsigned int B3D36 :1;
        unsigned int B3D37 :1;
    };
} B3D3bits_t;
extern volatile B3D3bits_t B3D3bits __attribute__((address(0xEB9)));
# 12500 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B3D4 __attribute__((address(0xEBA)));

__asm("B3D4 equ 0EBAh");


typedef union {
    struct {
        unsigned int B3D4 :8;
    };
    struct {
        unsigned int B3D40 :1;
        unsigned int B3D41 :1;
        unsigned int B3D42 :1;
        unsigned int B3D43 :1;
        unsigned int B3D44 :1;
        unsigned int B3D45 :1;
        unsigned int B3D46 :1;
        unsigned int B3D47 :1;
    };
} B3D4bits_t;
extern volatile B3D4bits_t B3D4bits __attribute__((address(0xEBA)));
# 12570 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B3D5 __attribute__((address(0xEBB)));

__asm("B3D5 equ 0EBBh");


typedef union {
    struct {
        unsigned int B3D5 :8;
    };
    struct {
        unsigned int B3D50 :1;
        unsigned int B3D51 :1;
        unsigned int B3D52 :1;
        unsigned int B3D53 :1;
        unsigned int B3D54 :1;
        unsigned int B3D55 :1;
        unsigned int B3D56 :1;
        unsigned int B3D57 :1;
    };
} B3D5bits_t;
extern volatile B3D5bits_t B3D5bits __attribute__((address(0xEBB)));
# 12640 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B3D6 __attribute__((address(0xEBC)));

__asm("B3D6 equ 0EBCh");


typedef union {
    struct {
        unsigned int B3D6 :8;
    };
    struct {
        unsigned int B3D60 :1;
        unsigned int B3D61 :1;
        unsigned int B3D62 :1;
        unsigned int B3D63 :1;
        unsigned int B3D64 :1;
        unsigned int B3D65 :1;
        unsigned int B3D66 :1;
        unsigned int B3D67 :1;
    };
} B3D6bits_t;
extern volatile B3D6bits_t B3D6bits __attribute__((address(0xEBC)));
# 12710 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B3D7 __attribute__((address(0xEBD)));

__asm("B3D7 equ 0EBDh");


typedef union {
    struct {
        unsigned int B3D7 :8;
    };
    struct {
        unsigned int B3D70 :1;
        unsigned int B3D71 :1;
        unsigned int B3D72 :1;
        unsigned int B3D73 :1;
        unsigned int B3D74 :1;
        unsigned int B3D75 :1;
        unsigned int B3D76 :1;
        unsigned int B3D77 :1;
    };
} B3D7bits_t;
extern volatile B3D7bits_t B3D7bits __attribute__((address(0xEBD)));
# 12780 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CANSTAT_RO6 __attribute__((address(0xEBE)));

__asm("CANSTAT_RO6 equ 0EBEh");


typedef union {
    struct {
        unsigned int EICODE0 :1;
        unsigned int EICODE1_ICODE0 :1;
        unsigned int EICODE2_ICODE1 :1;
        unsigned int EICODE3_ICODE2 :1;
        unsigned int EICODE4 :1;
        unsigned int OPMODE :3;
    };
    struct {
        unsigned int :1;
        unsigned int EICODE1 :1;
        unsigned int EICODE2 :1;
        unsigned int EICODE3 :1;
        unsigned int :1;
        unsigned int OPMODE0 :1;
        unsigned int OPMODE1 :1;
        unsigned int OPMODE2 :1;
    };
    struct {
        unsigned int :1;
        unsigned int ICODE0 :1;
        unsigned int ICODE1 :1;
        unsigned int ICODE2 :1;
    };
} CANSTAT_RO6bits_t;
extern volatile CANSTAT_RO6bits_t CANSTAT_RO6bits __attribute__((address(0xEBE)));
# 12891 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CANCON_RO6 __attribute__((address(0xEBF)));

__asm("CANCON_RO6 equ 0EBFh");


typedef union {
    struct {
        unsigned int FP0 :1;
        unsigned int WIN0_FP1 :1;
        unsigned int WIN1_FP2 :1;
        unsigned int WIN2_FP3 :1;
        unsigned int ABAT :1;
        unsigned int REQOP :3;
    };
    struct {
        unsigned int :1;
        unsigned int WIN0 :1;
        unsigned int WIN1 :1;
        unsigned int WIN2 :1;
    };
    struct {
        unsigned int :1;
        unsigned int FP1 :1;
        unsigned int FP2 :1;
        unsigned int FP3 :1;
    };
} CANCON_RO6bits_t;
extern volatile CANCON_RO6bits_t CANCON_RO6bits __attribute__((address(0xEBF)));
# 12983 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B4CON __attribute__((address(0xEC0)));

__asm("B4CON equ 0EC0h");


typedef union {
    struct {
        unsigned int FILHIT0_TXPRI0 :1;
        unsigned int FILHIT1_TXPRI1 :1;
        unsigned int FILHIT2_RTREN :1;
        unsigned int FILHIT3_TXREQ :1;
        unsigned int FILHIT4_TXERR :1;
        unsigned int RXRTRRO_TXLARB :1;
        unsigned int RXM1_TXABT :1;
        unsigned int RXFUL_TXBIF :1;
    };
    struct {
        unsigned int FILHIT0 :1;
        unsigned int FILHIT1 :1;
        unsigned int FILHIT2 :1;
        unsigned int FILHIT3 :1;
        unsigned int FILHIT4 :1;
        unsigned int RXRTRRO :1;
        unsigned int RXM1 :1;
        unsigned int RXFUL :1;
    };
    struct {
        unsigned int TXPRI0 :1;
        unsigned int TXPRI1 :1;
        unsigned int RTREN :1;
        unsigned int TXREQ :1;
        unsigned int TXERR :1;
        unsigned int TXLARB :1;
        unsigned int TXABT :1;
        unsigned int TXBIF :1;
    };
    struct {
        unsigned int B4FILHIT0 :1;
        unsigned int B4FILHIT1 :1;
        unsigned int B4FILHIT2 :1;
        unsigned int B4FILHIT3 :1;
        unsigned int B4FILHIT4 :1;
        unsigned int B4RTRRO :1;
        unsigned int B4RXM1 :1;
        unsigned int B4RXFUL :1;
    };
    struct {
        unsigned int B4TXPRI0 :1;
        unsigned int B4TXPRI1 :1;
        unsigned int B4RTREN :1;
        unsigned int B4TXREQ :1;
        unsigned int B4TXERR :1;
        unsigned int B4TXLARB :1;
        unsigned int B4TXABT :1;
        unsigned int B4TXB3IF :1;
    };
} B4CONbits_t;
extern volatile B4CONbits_t B4CONbits __attribute__((address(0xEC0)));
# 13245 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B4SIDH __attribute__((address(0xEC1)));

__asm("B4SIDH equ 0EC1h");


typedef union {
    struct {
        unsigned int SID :8;
    };
    struct {
        unsigned int SID3 :1;
        unsigned int SID4 :1;
        unsigned int SID5 :1;
        unsigned int SID6 :1;
        unsigned int SID7 :1;
        unsigned int SID8 :1;
        unsigned int SID9 :1;
        unsigned int SID10 :1;
    };
    struct {
        unsigned int B4SID3 :1;
        unsigned int B4SID4 :1;
        unsigned int B4SID5 :1;
        unsigned int B4SID6 :1;
        unsigned int B4SID7 :1;
        unsigned int B4SID8 :1;
        unsigned int B4SID9 :1;
        unsigned int B4SID10 :1;
    };
} B4SIDHbits_t;
extern volatile B4SIDHbits_t B4SIDHbits __attribute__((address(0xEC1)));
# 13365 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B4SIDL __attribute__((address(0xEC2)));

__asm("B4SIDL equ 0EC2h");


typedef union {
    struct {
        unsigned int EID :2;
        unsigned int :1;
        unsigned int EXIDE :1;
        unsigned int SRR :1;
        unsigned int SID :3;
    };
    struct {
        unsigned int EID16 :1;
        unsigned int EID17 :1;
        unsigned int :3;
        unsigned int SID0 :1;
        unsigned int SID1 :1;
        unsigned int SID2 :1;
    };
    struct {
        unsigned int B4EID16 :1;
        unsigned int B4EID17 :1;
        unsigned int :1;
        unsigned int B4EXID :1;
        unsigned int B4SRR :1;
        unsigned int B4SID0 :1;
        unsigned int B4SID1 :1;
        unsigned int B4SID2 :1;
    };
    struct {
        unsigned int :3;
        unsigned int B4EXIDE :1;
    };
} B4SIDLbits_t;
extern volatile B4SIDLbits_t B4SIDLbits __attribute__((address(0xEC2)));
# 13491 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B4EIDH __attribute__((address(0xEC3)));

__asm("B4EIDH equ 0EC3h");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID8 :1;
        unsigned int EID9 :1;
        unsigned int EID10 :1;
        unsigned int EID11 :1;
        unsigned int EID12 :1;
        unsigned int EID13 :1;
        unsigned int EID14 :1;
        unsigned int EID15 :1;
    };
    struct {
        unsigned int B4EID8 :1;
        unsigned int B4EID9 :1;
        unsigned int B4EID10 :1;
        unsigned int B4EID11 :1;
        unsigned int B4EID12 :1;
        unsigned int B4EID13 :1;
        unsigned int B4EID14 :1;
        unsigned int B4EID15 :1;
    };
} B4EIDHbits_t;
extern volatile B4EIDHbits_t B4EIDHbits __attribute__((address(0xEC3)));
# 13611 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B4EIDL __attribute__((address(0xEC4)));

__asm("B4EIDL equ 0EC4h");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID0 :1;
        unsigned int EID1 :1;
        unsigned int EID2 :1;
        unsigned int EID3 :1;
        unsigned int EID4 :1;
        unsigned int EID5 :1;
        unsigned int EID6 :1;
        unsigned int EID7 :1;
    };
    struct {
        unsigned int B4EID0 :1;
        unsigned int B4EID1 :1;
        unsigned int B4EID2 :1;
        unsigned int B4EID3 :1;
        unsigned int B4EID4 :1;
        unsigned int B4EID5 :1;
        unsigned int B4EID6 :1;
        unsigned int B4EID7 :1;
    };
} B4EIDLbits_t;
extern volatile B4EIDLbits_t B4EIDLbits __attribute__((address(0xEC4)));
# 13731 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B4DLC __attribute__((address(0xEC5)));

__asm("B4DLC equ 0EC5h");


typedef union {
    struct {
        unsigned int DLC :4;
        unsigned int RB :2;
        unsigned int RXRTR_TXRTR :1;
    };
    struct {
        unsigned int DLC0 :1;
        unsigned int DLC1 :1;
        unsigned int DLC2 :1;
        unsigned int DLC3 :1;
        unsigned int RB0 :1;
        unsigned int RB1 :1;
        unsigned int RXRTR :1;
    };
    struct {
        unsigned int :6;
        unsigned int TXRTR :1;
    };
    struct {
        unsigned int B4DLC0 :1;
        unsigned int B4DLC1 :1;
        unsigned int B4DLC2 :1;
        unsigned int B4DLC3 :1;
        unsigned int B4RB0 :1;
        unsigned int B4RB1 :1;
        unsigned int B4RXRTR :1;
    };
} B4DLCbits_t;
extern volatile B4DLCbits_t B4DLCbits __attribute__((address(0xEC5)));
# 13860 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B4D0 __attribute__((address(0xEC6)));

__asm("B4D0 equ 0EC6h");


typedef union {
    struct {
        unsigned int B4D0 :8;
    };
    struct {
        unsigned int B4D00 :1;
        unsigned int B4D01 :1;
        unsigned int B4D02 :1;
        unsigned int B4D03 :1;
        unsigned int B4D04 :1;
        unsigned int B4D05 :1;
        unsigned int B4D06 :1;
        unsigned int B4D07 :1;
    };
} B4D0bits_t;
extern volatile B4D0bits_t B4D0bits __attribute__((address(0xEC6)));
# 13930 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B4D1 __attribute__((address(0xEC7)));

__asm("B4D1 equ 0EC7h");


typedef union {
    struct {
        unsigned int B4D1 :8;
    };
    struct {
        unsigned int B4D10 :1;
        unsigned int B4D11 :1;
        unsigned int B4D12 :1;
        unsigned int B4D13 :1;
        unsigned int B4D14 :1;
        unsigned int B4D15 :1;
        unsigned int B4D16 :1;
        unsigned int B4D17 :1;
    };
} B4D1bits_t;
extern volatile B4D1bits_t B4D1bits __attribute__((address(0xEC7)));
# 14000 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B4D2 __attribute__((address(0xEC8)));

__asm("B4D2 equ 0EC8h");


typedef union {
    struct {
        unsigned int B4D2 :8;
    };
    struct {
        unsigned int B4D20 :1;
        unsigned int B4D21 :1;
        unsigned int B4D22 :1;
        unsigned int B4D23 :1;
        unsigned int B4D24 :1;
        unsigned int B4D25 :1;
        unsigned int B4D26 :1;
        unsigned int B4D27 :1;
    };
} B4D2bits_t;
extern volatile B4D2bits_t B4D2bits __attribute__((address(0xEC8)));
# 14070 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B4D3 __attribute__((address(0xEC9)));

__asm("B4D3 equ 0EC9h");


typedef union {
    struct {
        unsigned int B4D3 :8;
    };
    struct {
        unsigned int B4D30 :1;
        unsigned int B4D31 :1;
        unsigned int B4D32 :1;
        unsigned int B4D33 :1;
        unsigned int B4D34 :1;
        unsigned int B4D35 :1;
        unsigned int B4D36 :1;
        unsigned int B4D37 :1;
    };
} B4D3bits_t;
extern volatile B4D3bits_t B4D3bits __attribute__((address(0xEC9)));
# 14140 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B4D4 __attribute__((address(0xECA)));

__asm("B4D4 equ 0ECAh");


typedef union {
    struct {
        unsigned int B4D4 :8;
    };
    struct {
        unsigned int B4D40 :1;
        unsigned int B4D41 :1;
        unsigned int B4D42 :1;
        unsigned int B4D43 :1;
        unsigned int B4D44 :1;
        unsigned int B4D45 :1;
        unsigned int B4D46 :1;
        unsigned int B4D47 :1;
    };
} B4D4bits_t;
extern volatile B4D4bits_t B4D4bits __attribute__((address(0xECA)));
# 14210 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B4D5 __attribute__((address(0xECB)));

__asm("B4D5 equ 0ECBh");


typedef union {
    struct {
        unsigned int B4D5 :8;
    };
    struct {
        unsigned int B4D50 :1;
        unsigned int B4D51 :1;
        unsigned int B4D52 :1;
        unsigned int B4D53 :1;
        unsigned int B4D54 :1;
        unsigned int B4D55 :1;
        unsigned int B4D56 :1;
        unsigned int B4D57 :1;
    };
} B4D5bits_t;
extern volatile B4D5bits_t B4D5bits __attribute__((address(0xECB)));
# 14280 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B4D6 __attribute__((address(0xECC)));

__asm("B4D6 equ 0ECCh");


typedef union {
    struct {
        unsigned int B4D6 :8;
    };
    struct {
        unsigned int B4D60 :1;
        unsigned int B4D61 :1;
        unsigned int B4D62 :1;
        unsigned int B4D63 :1;
        unsigned int B4D64 :1;
        unsigned int B4D65 :1;
        unsigned int B4D66 :1;
        unsigned int B4D67 :1;
    };
} B4D6bits_t;
extern volatile B4D6bits_t B4D6bits __attribute__((address(0xECC)));
# 14350 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B4D7 __attribute__((address(0xECD)));

__asm("B4D7 equ 0ECDh");


typedef union {
    struct {
        unsigned int B4D7 :8;
    };
    struct {
        unsigned int B4D70 :1;
        unsigned int B4D71 :1;
        unsigned int B4D72 :1;
        unsigned int B4D73 :1;
        unsigned int B4D74 :1;
        unsigned int B4D75 :1;
        unsigned int B4D76 :1;
        unsigned int B4D77 :1;
    };
} B4D7bits_t;
extern volatile B4D7bits_t B4D7bits __attribute__((address(0xECD)));
# 14420 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CANSTAT_RO5 __attribute__((address(0xECE)));

__asm("CANSTAT_RO5 equ 0ECEh");


typedef union {
    struct {
        unsigned int EICODE0 :1;
        unsigned int EICODE1_ICODE0 :1;
        unsigned int EICODE2_ICODE1 :1;
        unsigned int EICODE3_ICODE2 :1;
        unsigned int EICODE4 :1;
        unsigned int OPMODE :3;
    };
    struct {
        unsigned int :1;
        unsigned int EICODE1 :1;
        unsigned int EICODE2 :1;
        unsigned int EICODE3 :1;
        unsigned int :1;
        unsigned int OPMODE0 :1;
        unsigned int OPMODE1 :1;
        unsigned int OPMODE2 :1;
    };
    struct {
        unsigned int :1;
        unsigned int ICODE0 :1;
        unsigned int ICODE1 :1;
        unsigned int ICODE2 :1;
    };
} CANSTAT_RO5bits_t;
extern volatile CANSTAT_RO5bits_t CANSTAT_RO5bits __attribute__((address(0xECE)));
# 14531 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CANCON_RO5 __attribute__((address(0xECF)));

__asm("CANCON_RO5 equ 0ECFh");


typedef union {
    struct {
        unsigned int FP0 :1;
        unsigned int WIN0_FP1 :1;
        unsigned int WIN1_FP2 :1;
        unsigned int WIN2_FP3 :1;
        unsigned int ABAT :1;
        unsigned int REQOP :3;
    };
    struct {
        unsigned int :1;
        unsigned int WIN0 :1;
        unsigned int WIN1 :1;
        unsigned int WIN2 :1;
    };
    struct {
        unsigned int :1;
        unsigned int FP1 :1;
        unsigned int FP2 :1;
        unsigned int FP3 :1;
    };
} CANCON_RO5bits_t;
extern volatile CANCON_RO5bits_t CANCON_RO5bits __attribute__((address(0xECF)));
# 14623 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B5CON __attribute__((address(0xED0)));

__asm("B5CON equ 0ED0h");


typedef union {
    struct {
        unsigned int FILHIT0_TXPRI0 :1;
        unsigned int FILHIT1_TXPRI1 :1;
        unsigned int FILHIT2_RTREN :1;
        unsigned int FILHIT3_TXREQ :1;
        unsigned int FILHIT4_TXERR :1;
        unsigned int RXRTRRO_TXLARB :1;
        unsigned int RXM1_TXABT :1;
        unsigned int RXFUL_TXBIF :1;
    };
    struct {
        unsigned int FILHIT0 :1;
        unsigned int FILHIT1 :1;
        unsigned int FILHIT2 :1;
        unsigned int FILHIT3 :1;
        unsigned int FILHIT4 :1;
        unsigned int RXRTRRO :1;
        unsigned int RXM1 :1;
        unsigned int RXFUL :1;
    };
    struct {
        unsigned int TXPRI0 :1;
        unsigned int TXPRI1 :1;
        unsigned int RTREN :1;
        unsigned int TXREQ :1;
        unsigned int TXERR :1;
        unsigned int TXLARB :1;
        unsigned int TXABT :1;
        unsigned int TXBIF :1;
    };
    struct {
        unsigned int B5FILHIT0 :1;
        unsigned int B5FILHIT1 :1;
        unsigned int B5FILHIT2 :1;
        unsigned int B5FILHIT3 :1;
        unsigned int B5FILHIT4 :1;
        unsigned int B5RTRRO :1;
        unsigned int B5RXM1 :1;
        unsigned int B5RXFUL :1;
    };
    struct {
        unsigned int B5TXPRI0 :1;
        unsigned int B5TXPRI1 :1;
        unsigned int B5RTREN :1;
        unsigned int B5TXREQ :1;
        unsigned int B5TXERR :1;
        unsigned int B5TXLARB :1;
        unsigned int B5TXABT :1;
        unsigned int B5TXBIF :1;
    };
} B5CONbits_t;
extern volatile B5CONbits_t B5CONbits __attribute__((address(0xED0)));
# 14885 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B5SIDH __attribute__((address(0xED1)));

__asm("B5SIDH equ 0ED1h");


typedef union {
    struct {
        unsigned int SID :8;
    };
    struct {
        unsigned int SID3 :1;
        unsigned int SID4 :1;
        unsigned int SID5 :1;
        unsigned int SID6 :1;
        unsigned int SID7 :1;
        unsigned int SID8 :1;
        unsigned int SID9 :1;
        unsigned int SID10 :1;
    };
    struct {
        unsigned int B5SID3 :1;
        unsigned int B5SID4 :1;
        unsigned int B5SID5 :1;
        unsigned int B5SID6 :1;
        unsigned int B5SID7 :1;
        unsigned int B5SID8 :1;
        unsigned int B5SID9 :1;
        unsigned int B5SID10 :1;
    };
} B5SIDHbits_t;
extern volatile B5SIDHbits_t B5SIDHbits __attribute__((address(0xED1)));
# 15005 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B5SIDL __attribute__((address(0xED2)));

__asm("B5SIDL equ 0ED2h");


typedef union {
    struct {
        unsigned int EID :2;
        unsigned int :1;
        unsigned int EXIDE :1;
        unsigned int SRR :1;
        unsigned int SID :3;
    };
    struct {
        unsigned int EID16 :1;
        unsigned int EID17 :1;
        unsigned int :3;
        unsigned int SID0 :1;
        unsigned int SID1 :1;
        unsigned int SID2 :1;
    };
    struct {
        unsigned int B5EID16 :1;
        unsigned int B5EID17 :1;
        unsigned int :1;
        unsigned int B5EXID :1;
        unsigned int B5SRR :1;
        unsigned int B5SID0 :1;
        unsigned int B5SID1 :1;
        unsigned int B5SID2 :1;
    };
    struct {
        unsigned int :3;
        unsigned int B5EXIDE :1;
    };
} B5SIDLbits_t;
extern volatile B5SIDLbits_t B5SIDLbits __attribute__((address(0xED2)));
# 15131 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B5EIDH __attribute__((address(0xED3)));

__asm("B5EIDH equ 0ED3h");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID8 :1;
        unsigned int EID9 :1;
        unsigned int EID10 :1;
        unsigned int EID11 :1;
        unsigned int EID12 :1;
        unsigned int EID13 :1;
        unsigned int EID14 :1;
        unsigned int EID15 :1;
    };
    struct {
        unsigned int B5EID8 :1;
        unsigned int B5EID9 :1;
        unsigned int B5EID10 :1;
        unsigned int B5EID11 :1;
        unsigned int B5EID12 :1;
        unsigned int B5EID13 :1;
        unsigned int B5EID14 :1;
        unsigned int B5EID15 :1;
    };
} B5EIDHbits_t;
extern volatile B5EIDHbits_t B5EIDHbits __attribute__((address(0xED3)));
# 15251 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B5EIDL __attribute__((address(0xED4)));

__asm("B5EIDL equ 0ED4h");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID0 :1;
        unsigned int EID1 :1;
        unsigned int EID2 :1;
        unsigned int EID3 :1;
        unsigned int EID4 :1;
        unsigned int EID5 :1;
        unsigned int EID6 :1;
        unsigned int EID7 :1;
    };
    struct {
        unsigned int B5EID0 :1;
        unsigned int B5EID1 :1;
        unsigned int B5EID2 :1;
        unsigned int B5EID3 :1;
        unsigned int B5EID4 :1;
        unsigned int B5EID5 :1;
        unsigned int B5EID6 :1;
        unsigned int B5EID7 :1;
    };
} B5EIDLbits_t;
extern volatile B5EIDLbits_t B5EIDLbits __attribute__((address(0xED4)));
# 15371 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B5DLC __attribute__((address(0xED5)));

__asm("B5DLC equ 0ED5h");


typedef union {
    struct {
        unsigned int DLC :4;
        unsigned int RB :2;
        unsigned int RXRTR_TXRTR :1;
    };
    struct {
        unsigned int DLC0 :1;
        unsigned int DLC1 :1;
        unsigned int DLC2 :1;
        unsigned int DLC3 :1;
        unsigned int RB0 :1;
        unsigned int RB1 :1;
        unsigned int RXRTR :1;
    };
    struct {
        unsigned int :6;
        unsigned int TXRTR :1;
    };
    struct {
        unsigned int B5DLC0 :1;
        unsigned int B5DLC1 :1;
        unsigned int B5DLC2 :1;
        unsigned int B5DLC3 :1;
        unsigned int B5RB0 :1;
        unsigned int B5RB1 :1;
        unsigned int B5RXRTR :1;
    };
} B5DLCbits_t;
extern volatile B5DLCbits_t B5DLCbits __attribute__((address(0xED5)));
# 15500 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B5D0 __attribute__((address(0xED6)));

__asm("B5D0 equ 0ED6h");


typedef union {
    struct {
        unsigned int B5D0 :8;
    };
    struct {
        unsigned int B5D00 :1;
        unsigned int B5D01 :1;
        unsigned int B5D02 :1;
        unsigned int B5D03 :1;
        unsigned int B5D04 :1;
        unsigned int B5D05 :1;
        unsigned int B5D06 :1;
        unsigned int B5D07 :1;
    };
} B5D0bits_t;
extern volatile B5D0bits_t B5D0bits __attribute__((address(0xED6)));
# 15570 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B5D1 __attribute__((address(0xED7)));

__asm("B5D1 equ 0ED7h");


typedef union {
    struct {
        unsigned int B5D1 :8;
    };
    struct {
        unsigned int B5D10 :1;
        unsigned int B5D11 :1;
        unsigned int B5D12 :1;
        unsigned int B5D13 :1;
        unsigned int B5D14 :1;
        unsigned int B5D15 :1;
        unsigned int B5D16 :1;
        unsigned int B5D17 :1;
    };
} B5D1bits_t;
extern volatile B5D1bits_t B5D1bits __attribute__((address(0xED7)));
# 15640 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B5D2 __attribute__((address(0xED8)));

__asm("B5D2 equ 0ED8h");


typedef union {
    struct {
        unsigned int B5D2 :8;
    };
    struct {
        unsigned int B5D20 :1;
        unsigned int B5D21 :1;
        unsigned int B5D22 :1;
        unsigned int B5D23 :1;
        unsigned int B5D24 :1;
        unsigned int B5D25 :1;
        unsigned int B5D26 :1;
        unsigned int B5D27 :1;
    };
} B5D2bits_t;
extern volatile B5D2bits_t B5D2bits __attribute__((address(0xED8)));
# 15710 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B5D3 __attribute__((address(0xED9)));

__asm("B5D3 equ 0ED9h");


typedef union {
    struct {
        unsigned int B5D3 :8;
    };
    struct {
        unsigned int B5D30 :1;
        unsigned int B5D31 :1;
        unsigned int B5D32 :1;
        unsigned int B5D33 :1;
        unsigned int B5D34 :1;
        unsigned int B5D35 :1;
        unsigned int B5D36 :1;
        unsigned int B5D37 :1;
    };
} B5D3bits_t;
extern volatile B5D3bits_t B5D3bits __attribute__((address(0xED9)));
# 15780 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B5D4 __attribute__((address(0xEDA)));

__asm("B5D4 equ 0EDAh");


typedef union {
    struct {
        unsigned int B5D4 :8;
    };
    struct {
        unsigned int B5D40 :1;
        unsigned int B5D41 :1;
        unsigned int B5D42 :1;
        unsigned int B5D43 :1;
        unsigned int B5D44 :1;
        unsigned int B5D45 :1;
        unsigned int B5D46 :1;
        unsigned int B5D47 :1;
    };
} B5D4bits_t;
extern volatile B5D4bits_t B5D4bits __attribute__((address(0xEDA)));
# 15850 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B5D5 __attribute__((address(0xEDB)));

__asm("B5D5 equ 0EDBh");


typedef union {
    struct {
        unsigned int B5D5 :8;
    };
    struct {
        unsigned int B5D50 :1;
        unsigned int B5D51 :1;
        unsigned int B5D52 :1;
        unsigned int B5D53 :1;
        unsigned int B5D54 :1;
        unsigned int B5D55 :1;
        unsigned int B5D56 :1;
        unsigned int B5D57 :1;
    };
} B5D5bits_t;
extern volatile B5D5bits_t B5D5bits __attribute__((address(0xEDB)));
# 15920 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B5D6 __attribute__((address(0xEDC)));

__asm("B5D6 equ 0EDCh");


typedef union {
    struct {
        unsigned int B5D6 :8;
    };
    struct {
        unsigned int B5D60 :1;
        unsigned int B5D61 :1;
        unsigned int B5D62 :1;
        unsigned int B5D63 :1;
        unsigned int B5D64 :1;
        unsigned int B5D65 :1;
        unsigned int B5D66 :1;
        unsigned int B5D67 :1;
    };
} B5D6bits_t;
extern volatile B5D6bits_t B5D6bits __attribute__((address(0xEDC)));
# 15990 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char B5D7 __attribute__((address(0xEDD)));

__asm("B5D7 equ 0EDDh");


typedef union {
    struct {
        unsigned int B5D7 :8;
    };
    struct {
        unsigned int B5D70 :1;
        unsigned int B5D71 :1;
        unsigned int B5D72 :1;
        unsigned int B5D73 :1;
        unsigned int B5D74 :1;
        unsigned int B5D75 :1;
        unsigned int B5D76 :1;
        unsigned int B5D77 :1;
    };
} B5D7bits_t;
extern volatile B5D7bits_t B5D7bits __attribute__((address(0xEDD)));
# 16060 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CANSTAT_RO4 __attribute__((address(0xEDE)));

__asm("CANSTAT_RO4 equ 0EDEh");


typedef union {
    struct {
        unsigned int EICODE0 :1;
        unsigned int EICODE1_ICODE0 :1;
        unsigned int EICODE2_ICODE1 :1;
        unsigned int EICODE3_ICODE2 :1;
        unsigned int EICODE4 :1;
        unsigned int OPMODE :3;
    };
    struct {
        unsigned int :1;
        unsigned int EICODE1 :1;
        unsigned int EICODE2 :1;
        unsigned int EICODE3 :1;
        unsigned int :1;
        unsigned int OPMODE0 :1;
        unsigned int OPMODE1 :1;
        unsigned int OPMODE2 :1;
    };
    struct {
        unsigned int :1;
        unsigned int ICODE0 :1;
        unsigned int ICODE1 :1;
        unsigned int ICODE2 :1;
    };
} CANSTAT_RO4bits_t;
extern volatile CANSTAT_RO4bits_t CANSTAT_RO4bits __attribute__((address(0xEDE)));
# 16171 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CANCON_RO4 __attribute__((address(0xEDF)));

__asm("CANCON_RO4 equ 0EDFh");


typedef union {
    struct {
        unsigned int FP0 :1;
        unsigned int WIN0_FP1 :1;
        unsigned int WIN1_FP2 :1;
        unsigned int WIN2_FP3 :1;
        unsigned int ABAT :1;
        unsigned int REQOP :3;
    };
    struct {
        unsigned int :1;
        unsigned int WIN0 :1;
        unsigned int WIN1 :1;
        unsigned int WIN2 :1;
    };
    struct {
        unsigned int :1;
        unsigned int FP1 :1;
        unsigned int FP2 :1;
        unsigned int FP3 :1;
    };
} CANCON_RO4bits_t;
extern volatile CANCON_RO4bits_t CANCON_RO4bits __attribute__((address(0xEDF)));
# 16263 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF0SIDH __attribute__((address(0xEE0)));

__asm("RXF0SIDH equ 0EE0h");


typedef union {
    struct {
        unsigned int SID :8;
    };
    struct {
        unsigned int SID3 :1;
        unsigned int SID4 :1;
        unsigned int SID5 :1;
        unsigned int SID6 :1;
        unsigned int SID7 :1;
        unsigned int SID8 :1;
        unsigned int SID9 :1;
        unsigned int SID10 :1;
    };
    struct {
        unsigned int RXF0SID3 :1;
        unsigned int RXF0SID4 :1;
        unsigned int RXF0SID5 :1;
        unsigned int RXF0SID6 :1;
        unsigned int RXF0SID7 :1;
        unsigned int RXF0SID8 :1;
        unsigned int RXF0SID9 :1;
        unsigned int RXF0SID10 :1;
    };
} RXF0SIDHbits_t;
extern volatile RXF0SIDHbits_t RXF0SIDHbits __attribute__((address(0xEE0)));
# 16383 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF0SIDL __attribute__((address(0xEE1)));

__asm("RXF0SIDL equ 0EE1h");


typedef union {
    struct {
        unsigned int EID :2;
        unsigned int :1;
        unsigned int EXIDEN :1;
        unsigned int :1;
        unsigned int SID :3;
    };
    struct {
        unsigned int EID16 :1;
        unsigned int EID17 :1;
        unsigned int :3;
        unsigned int SID0 :1;
        unsigned int SID1 :1;
        unsigned int SID2 :1;
    };
    struct {
        unsigned int RXF0EID16 :1;
        unsigned int RXF0EID17 :1;
        unsigned int :1;
        unsigned int RXF0EXIDEN :1;
        unsigned int :1;
        unsigned int RXF0SID0 :1;
        unsigned int RXF0SID1 :1;
        unsigned int RXF0SID2 :1;
    };
} RXF0SIDLbits_t;
extern volatile RXF0SIDLbits_t RXF0SIDLbits __attribute__((address(0xEE1)));
# 16490 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF0EIDH __attribute__((address(0xEE2)));

__asm("RXF0EIDH equ 0EE2h");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID8 :1;
        unsigned int EID9 :1;
        unsigned int EID10 :1;
        unsigned int EID11 :1;
        unsigned int EID12 :1;
        unsigned int EID13 :1;
        unsigned int EID14 :1;
        unsigned int EID15 :1;
    };
    struct {
        unsigned int RXF0EID8 :1;
        unsigned int RXF0EID9 :1;
        unsigned int RXF0EID10 :1;
        unsigned int RXF0EID11 :1;
        unsigned int RXF0EID12 :1;
        unsigned int RXF0EID13 :1;
        unsigned int RXF0EID14 :1;
        unsigned int RXF0EID15 :1;
    };
} RXF0EIDHbits_t;
extern volatile RXF0EIDHbits_t RXF0EIDHbits __attribute__((address(0xEE2)));
# 16610 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF0EIDL __attribute__((address(0xEE3)));

__asm("RXF0EIDL equ 0EE3h");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID0 :1;
        unsigned int EID1 :1;
        unsigned int EID2 :1;
        unsigned int EID3 :1;
        unsigned int EID4 :1;
        unsigned int EID5 :1;
        unsigned int EID6 :1;
        unsigned int EID7 :1;
    };
    struct {
        unsigned int RXF0EID0 :1;
        unsigned int RXF0EID1 :1;
        unsigned int RXF0EID2 :1;
        unsigned int RXF0EID3 :1;
        unsigned int RXF0EID4 :1;
        unsigned int RXF0EID5 :1;
        unsigned int RXF0EID6 :1;
        unsigned int RXF0EID7 :1;
    };
} RXF0EIDLbits_t;
extern volatile RXF0EIDLbits_t RXF0EIDLbits __attribute__((address(0xEE3)));
# 16730 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF1SIDH __attribute__((address(0xEE4)));

__asm("RXF1SIDH equ 0EE4h");


typedef union {
    struct {
        unsigned int SID :8;
    };
    struct {
        unsigned int SID3 :1;
        unsigned int SID4 :1;
        unsigned int SID5 :1;
        unsigned int SID6 :1;
        unsigned int SID7 :1;
        unsigned int SID8 :1;
        unsigned int SID9 :1;
        unsigned int SID10 :1;
    };
    struct {
        unsigned int RXF1SID3 :1;
        unsigned int RXF1SID4 :1;
        unsigned int RXF1SID5 :1;
        unsigned int RXF1SID6 :1;
        unsigned int RXF1SID7 :1;
        unsigned int RXF1SID8 :1;
        unsigned int RXF1SID9 :1;
        unsigned int RXF1SID10 :1;
    };
} RXF1SIDHbits_t;
extern volatile RXF1SIDHbits_t RXF1SIDHbits __attribute__((address(0xEE4)));
# 16850 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF1SIDL __attribute__((address(0xEE5)));

__asm("RXF1SIDL equ 0EE5h");


typedef union {
    struct {
        unsigned int EID :2;
        unsigned int :1;
        unsigned int EXIDEN :1;
        unsigned int :1;
        unsigned int SID :3;
    };
    struct {
        unsigned int EID16 :1;
        unsigned int EID17 :1;
        unsigned int :3;
        unsigned int SID0 :1;
        unsigned int SID1 :1;
        unsigned int SID2 :1;
    };
    struct {
        unsigned int RXF1EID16 :1;
        unsigned int RXF1EID17 :1;
        unsigned int :1;
        unsigned int RXF1EXIDEN :1;
        unsigned int :1;
        unsigned int RXF1SID0 :1;
        unsigned int RXF1SID1 :1;
        unsigned int RXF1SID2 :1;
    };
} RXF1SIDLbits_t;
extern volatile RXF1SIDLbits_t RXF1SIDLbits __attribute__((address(0xEE5)));
# 16957 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF1EIDH __attribute__((address(0xEE6)));

__asm("RXF1EIDH equ 0EE6h");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID8 :1;
        unsigned int EID9 :1;
        unsigned int EID10 :1;
        unsigned int EID11 :1;
        unsigned int EID12 :1;
        unsigned int EID13 :1;
        unsigned int EID14 :1;
        unsigned int EID15 :1;
    };
    struct {
        unsigned int RXF1EID8 :1;
        unsigned int RXF1EID9 :1;
        unsigned int RXF1EID10 :1;
        unsigned int RXF1EID11 :1;
        unsigned int RXF1EID12 :1;
        unsigned int RXF1EID13 :1;
        unsigned int RXF1EID14 :1;
        unsigned int RXF1EID15 :1;
    };
} RXF1EIDHbits_t;
extern volatile RXF1EIDHbits_t RXF1EIDHbits __attribute__((address(0xEE6)));
# 17077 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF1EIDL __attribute__((address(0xEE7)));

__asm("RXF1EIDL equ 0EE7h");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID0 :1;
        unsigned int EID1 :1;
        unsigned int EID2 :1;
        unsigned int EID3 :1;
        unsigned int EID4 :1;
        unsigned int EID5 :1;
        unsigned int EID6 :1;
        unsigned int EID7 :1;
    };
    struct {
        unsigned int RXF1EID0 :1;
        unsigned int RXF1EID1 :1;
        unsigned int RXF1EID2 :1;
        unsigned int RXF1EID3 :1;
        unsigned int RXF1EID4 :1;
        unsigned int RXF1EID5 :1;
        unsigned int RXF1EID6 :1;
        unsigned int RXF1EID7 :1;
    };
} RXF1EIDLbits_t;
extern volatile RXF1EIDLbits_t RXF1EIDLbits __attribute__((address(0xEE7)));
# 17197 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF2SIDH __attribute__((address(0xEE8)));

__asm("RXF2SIDH equ 0EE8h");


typedef union {
    struct {
        unsigned int SID :8;
    };
    struct {
        unsigned int SID3 :1;
        unsigned int SID4 :1;
        unsigned int SID5 :1;
        unsigned int SID6 :1;
        unsigned int SID7 :1;
        unsigned int SID8 :1;
        unsigned int SID9 :1;
        unsigned int SID10 :1;
    };
    struct {
        unsigned int RXF2SID3 :1;
        unsigned int RXF2SID4 :1;
        unsigned int RXF2SID5 :1;
        unsigned int RXF2SID6 :1;
        unsigned int RXF2SID7 :1;
        unsigned int RXF2SID8 :1;
        unsigned int RXF2SID9 :1;
        unsigned int RXF2SID10 :1;
    };
} RXF2SIDHbits_t;
extern volatile RXF2SIDHbits_t RXF2SIDHbits __attribute__((address(0xEE8)));
# 17317 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF2SIDL __attribute__((address(0xEE9)));

__asm("RXF2SIDL equ 0EE9h");


typedef union {
    struct {
        unsigned int EID :2;
        unsigned int :1;
        unsigned int EXIDEN :1;
        unsigned int :1;
        unsigned int SID :3;
    };
    struct {
        unsigned int EID16 :1;
        unsigned int EID17 :1;
        unsigned int :3;
        unsigned int SID0 :1;
        unsigned int SID1 :1;
        unsigned int SID2 :1;
    };
    struct {
        unsigned int RXF2EID16 :1;
        unsigned int RXF2EID17 :1;
        unsigned int :1;
        unsigned int RXF2EXIDEN :1;
        unsigned int :1;
        unsigned int RXF2SID0 :1;
        unsigned int RXF2SID1 :1;
        unsigned int RXF2SID2 :1;
    };
} RXF2SIDLbits_t;
extern volatile RXF2SIDLbits_t RXF2SIDLbits __attribute__((address(0xEE9)));
# 17424 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF2EIDH __attribute__((address(0xEEA)));

__asm("RXF2EIDH equ 0EEAh");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID8 :1;
        unsigned int EID9 :1;
        unsigned int EID10 :1;
        unsigned int EID11 :1;
        unsigned int EID12 :1;
        unsigned int EID13 :1;
        unsigned int EID14 :1;
        unsigned int EID15 :1;
    };
    struct {
        unsigned int RXF2EID8 :1;
        unsigned int RXF2EID9 :1;
        unsigned int RXF2EID10 :1;
        unsigned int RXF2EID11 :1;
        unsigned int RXF2EID12 :1;
        unsigned int RXF2EID13 :1;
        unsigned int RXF2EID14 :1;
        unsigned int RXF2EID15 :1;
    };
} RXF2EIDHbits_t;
extern volatile RXF2EIDHbits_t RXF2EIDHbits __attribute__((address(0xEEA)));
# 17544 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF2EIDL __attribute__((address(0xEEB)));

__asm("RXF2EIDL equ 0EEBh");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID0 :1;
        unsigned int EID1 :1;
        unsigned int EID2 :1;
        unsigned int EID3 :1;
        unsigned int EID4 :1;
        unsigned int EID5 :1;
        unsigned int EID6 :1;
        unsigned int EID7 :1;
    };
    struct {
        unsigned int RXF2EID0 :1;
        unsigned int RXF2EID1 :1;
        unsigned int RXF2EID2 :1;
        unsigned int RXF2EID3 :1;
        unsigned int RXF2EID4 :1;
        unsigned int RXF2EID5 :1;
        unsigned int RXF2EID6 :1;
        unsigned int RXF2EID7 :1;
    };
} RXF2EIDLbits_t;
extern volatile RXF2EIDLbits_t RXF2EIDLbits __attribute__((address(0xEEB)));
# 17664 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF3SIDH __attribute__((address(0xEEC)));

__asm("RXF3SIDH equ 0EECh");


typedef union {
    struct {
        unsigned int SID :8;
    };
    struct {
        unsigned int SID3 :1;
        unsigned int SID4 :1;
        unsigned int SID5 :1;
        unsigned int SID6 :1;
        unsigned int SID7 :1;
        unsigned int SID8 :1;
        unsigned int SID9 :1;
        unsigned int SID10 :1;
    };
    struct {
        unsigned int RXF3SID3 :1;
        unsigned int RXF3SID4 :1;
        unsigned int RXF3SID5 :1;
        unsigned int RXF3SID6 :1;
        unsigned int RXF3SID7 :1;
        unsigned int RXF3SID8 :1;
        unsigned int RXF3SID9 :1;
        unsigned int RXF3SID10 :1;
    };
} RXF3SIDHbits_t;
extern volatile RXF3SIDHbits_t RXF3SIDHbits __attribute__((address(0xEEC)));
# 17784 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF3SIDL __attribute__((address(0xEED)));

__asm("RXF3SIDL equ 0EEDh");


typedef union {
    struct {
        unsigned int EID :2;
        unsigned int :1;
        unsigned int EXIDEN :1;
        unsigned int :1;
        unsigned int SID :3;
    };
    struct {
        unsigned int EID16 :1;
        unsigned int EID17 :1;
        unsigned int :3;
        unsigned int SID0 :1;
        unsigned int SID1 :1;
        unsigned int SID2 :1;
    };
    struct {
        unsigned int RXF3EID16 :1;
        unsigned int RXF3EID17 :1;
        unsigned int :1;
        unsigned int RXF3EXIDEN :1;
        unsigned int :1;
        unsigned int RXF3SID0 :1;
        unsigned int RXF3SID1 :1;
        unsigned int RXF3SID2 :1;
    };
} RXF3SIDLbits_t;
extern volatile RXF3SIDLbits_t RXF3SIDLbits __attribute__((address(0xEED)));
# 17891 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF3EIDH __attribute__((address(0xEEE)));

__asm("RXF3EIDH equ 0EEEh");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID8 :1;
        unsigned int EID9 :1;
        unsigned int EID10 :1;
        unsigned int EID11 :1;
        unsigned int EID12 :1;
        unsigned int EID13 :1;
        unsigned int EID14 :1;
        unsigned int EID15 :1;
    };
    struct {
        unsigned int RXF3EID8 :1;
        unsigned int RXF3EID9 :1;
        unsigned int RXF3EID10 :1;
        unsigned int RXF3EID11 :1;
        unsigned int RXF3EID12 :1;
        unsigned int RXF3EID13 :1;
        unsigned int RXF3EID14 :1;
        unsigned int RXF3EID15 :1;
    };
} RXF3EIDHbits_t;
extern volatile RXF3EIDHbits_t RXF3EIDHbits __attribute__((address(0xEEE)));
# 18011 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF3EIDL __attribute__((address(0xEEF)));

__asm("RXF3EIDL equ 0EEFh");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID0 :1;
        unsigned int EID1 :1;
        unsigned int EID2 :1;
        unsigned int EID3 :1;
        unsigned int EID4 :1;
        unsigned int EID5 :1;
        unsigned int EID6 :1;
        unsigned int EID7 :1;
    };
    struct {
        unsigned int RXF3EID0 :1;
        unsigned int RXF3EID1 :1;
        unsigned int RXF3EID2 :1;
        unsigned int RXF3EID3 :1;
        unsigned int RXF3EID4 :1;
        unsigned int RXF3EID5 :1;
        unsigned int RXF3EID6 :1;
        unsigned int RXF3EID7 :1;
    };
} RXF3EIDLbits_t;
extern volatile RXF3EIDLbits_t RXF3EIDLbits __attribute__((address(0xEEF)));
# 18131 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF4SIDH __attribute__((address(0xEF0)));

__asm("RXF4SIDH equ 0EF0h");


typedef union {
    struct {
        unsigned int SID :8;
    };
    struct {
        unsigned int SID3 :1;
        unsigned int SID4 :1;
        unsigned int SID5 :1;
        unsigned int SID6 :1;
        unsigned int SID7 :1;
        unsigned int SID8 :1;
        unsigned int SID9 :1;
        unsigned int SID10 :1;
    };
    struct {
        unsigned int RXF4SID3 :1;
        unsigned int RXF4SID4 :1;
        unsigned int RXF4SID5 :1;
        unsigned int RXF4SID6 :1;
        unsigned int RXF4SID7 :1;
        unsigned int RXF4SID8 :1;
        unsigned int RXF4SID9 :1;
        unsigned int RXF4SID10 :1;
    };
} RXF4SIDHbits_t;
extern volatile RXF4SIDHbits_t RXF4SIDHbits __attribute__((address(0xEF0)));
# 18251 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF4SIDL __attribute__((address(0xEF1)));

__asm("RXF4SIDL equ 0EF1h");


typedef union {
    struct {
        unsigned int EID :2;
        unsigned int :1;
        unsigned int EXIDEN :1;
        unsigned int :1;
        unsigned int SID :3;
    };
    struct {
        unsigned int EID16 :1;
        unsigned int EID17 :1;
        unsigned int :3;
        unsigned int SID0 :1;
        unsigned int SID1 :1;
        unsigned int SID2 :1;
    };
    struct {
        unsigned int RXF4EID16 :1;
        unsigned int RXF4EID17 :1;
        unsigned int :1;
        unsigned int RXF4EXIDEN :1;
        unsigned int :1;
        unsigned int RXF4SID0 :1;
        unsigned int RXF4SID1 :1;
        unsigned int RXF4SID2 :1;
    };
} RXF4SIDLbits_t;
extern volatile RXF4SIDLbits_t RXF4SIDLbits __attribute__((address(0xEF1)));
# 18358 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF4EIDH __attribute__((address(0xEF2)));

__asm("RXF4EIDH equ 0EF2h");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID8 :1;
        unsigned int EID9 :1;
        unsigned int EID10 :1;
        unsigned int EID11 :1;
        unsigned int EID12 :1;
        unsigned int EID13 :1;
        unsigned int EID14 :1;
        unsigned int EID15 :1;
    };
    struct {
        unsigned int RXF4EID8 :1;
        unsigned int RXF4EID9 :1;
        unsigned int RXF4EID10 :1;
        unsigned int RXF4EID11 :1;
        unsigned int RXF4EID12 :1;
        unsigned int RXF4EID13 :1;
        unsigned int RXF4EID14 :1;
        unsigned int RXF4EID15 :1;
    };
} RXF4EIDHbits_t;
extern volatile RXF4EIDHbits_t RXF4EIDHbits __attribute__((address(0xEF2)));
# 18478 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF4EIDL __attribute__((address(0xEF3)));

__asm("RXF4EIDL equ 0EF3h");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID0 :1;
        unsigned int EID1 :1;
        unsigned int EID2 :1;
        unsigned int EID3 :1;
        unsigned int EID4 :1;
        unsigned int EID5 :1;
        unsigned int EID6 :1;
        unsigned int EID7 :1;
    };
    struct {
        unsigned int RXF4EID0 :1;
        unsigned int RXF4EID1 :1;
        unsigned int RXF4EID2 :1;
        unsigned int RXF4EID3 :1;
        unsigned int RXF4EID4 :1;
        unsigned int RXF4EID5 :1;
        unsigned int RXF4EID6 :1;
        unsigned int RXF4EID7 :1;
    };
} RXF4EIDLbits_t;
extern volatile RXF4EIDLbits_t RXF4EIDLbits __attribute__((address(0xEF3)));
# 18598 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF5SIDH __attribute__((address(0xEF4)));

__asm("RXF5SIDH equ 0EF4h");


typedef union {
    struct {
        unsigned int SID :8;
    };
    struct {
        unsigned int SID3 :1;
        unsigned int SID4 :1;
        unsigned int SID5 :1;
        unsigned int SID6 :1;
        unsigned int SID7 :1;
        unsigned int SID8 :1;
        unsigned int SID9 :1;
        unsigned int SID10 :1;
    };
    struct {
        unsigned int RXF5SID3 :1;
        unsigned int RXF5SID4 :1;
        unsigned int RXF5SID5 :1;
        unsigned int RXF5SID6 :1;
        unsigned int RXF5SID7 :1;
        unsigned int RXF5SID8 :1;
        unsigned int RXF5SID9 :1;
        unsigned int RXF5SID10 :1;
    };
} RXF5SIDHbits_t;
extern volatile RXF5SIDHbits_t RXF5SIDHbits __attribute__((address(0xEF4)));
# 18718 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF5SIDL __attribute__((address(0xEF5)));

__asm("RXF5SIDL equ 0EF5h");


typedef union {
    struct {
        unsigned int EID :2;
        unsigned int :1;
        unsigned int EXIDEN :1;
        unsigned int :1;
        unsigned int SID :3;
    };
    struct {
        unsigned int EID16 :1;
        unsigned int EID17 :1;
        unsigned int :3;
        unsigned int SID0 :1;
        unsigned int SID1 :1;
        unsigned int SID2 :1;
    };
    struct {
        unsigned int RXF5EID16 :1;
        unsigned int RXF5EID17 :1;
        unsigned int :1;
        unsigned int RXF5EXIDEN :1;
        unsigned int :1;
        unsigned int RXF5SID0 :1;
        unsigned int RXF5SID1 :1;
        unsigned int RXF5SID2 :1;
    };
} RXF5SIDLbits_t;
extern volatile RXF5SIDLbits_t RXF5SIDLbits __attribute__((address(0xEF5)));
# 18825 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF5EIDH __attribute__((address(0xEF6)));

__asm("RXF5EIDH equ 0EF6h");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID8 :1;
        unsigned int EID9 :1;
        unsigned int EID10 :1;
        unsigned int EID11 :1;
        unsigned int EID12 :1;
        unsigned int EID13 :1;
        unsigned int EID14 :1;
        unsigned int EID15 :1;
    };
    struct {
        unsigned int RXF5EID8 :1;
        unsigned int RXF5EID9 :1;
        unsigned int RXF5EID10 :1;
        unsigned int RXF5EID11 :1;
        unsigned int RXF5EID12 :1;
        unsigned int RXF5EID13 :1;
        unsigned int RXF5EID14 :1;
        unsigned int RXF5EID15 :1;
    };
} RXF5EIDHbits_t;
extern volatile RXF5EIDHbits_t RXF5EIDHbits __attribute__((address(0xEF6)));
# 18945 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXF5EIDL __attribute__((address(0xEF7)));

__asm("RXF5EIDL equ 0EF7h");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID0 :1;
        unsigned int EID1 :1;
        unsigned int EID2 :1;
        unsigned int EID3 :1;
        unsigned int EID4 :1;
        unsigned int EID5 :1;
        unsigned int EID6 :1;
        unsigned int EID7 :1;
    };
    struct {
        unsigned int RXF5EID0 :1;
        unsigned int RXF5EID1 :1;
        unsigned int RXF5EID2 :1;
        unsigned int RXF5EID3 :1;
        unsigned int RXF5EID4 :1;
        unsigned int RXF5EID5 :1;
        unsigned int RXF5EID6 :1;
        unsigned int RXF5EID7 :1;
    };
} RXF5EIDLbits_t;
extern volatile RXF5EIDLbits_t RXF5EIDLbits __attribute__((address(0xEF7)));
# 19065 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXM0SIDH __attribute__((address(0xEF8)));

__asm("RXM0SIDH equ 0EF8h");


typedef union {
    struct {
        unsigned int SID :8;
    };
    struct {
        unsigned int SID3 :1;
        unsigned int SID4 :1;
        unsigned int SID5 :1;
        unsigned int SID6 :1;
        unsigned int SID7 :1;
        unsigned int SID8 :1;
        unsigned int SID9 :1;
        unsigned int SID10 :1;
    };
    struct {
        unsigned int RXM0SID3 :1;
        unsigned int RXM0SID4 :1;
        unsigned int RXM0SID5 :1;
        unsigned int RXM0SID6 :1;
        unsigned int RXM0SID7 :1;
        unsigned int RXM0SID8 :1;
        unsigned int RXM0SID9 :1;
        unsigned int RXM0SID10 :1;
    };
} RXM0SIDHbits_t;
extern volatile RXM0SIDHbits_t RXM0SIDHbits __attribute__((address(0xEF8)));
# 19185 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXM0SIDL __attribute__((address(0xEF9)));

__asm("RXM0SIDL equ 0EF9h");


typedef union {
    struct {
        unsigned int EID :2;
        unsigned int :1;
        unsigned int EXIDEN :1;
        unsigned int :1;
        unsigned int SID :3;
    };
    struct {
        unsigned int EID16 :1;
        unsigned int EID17 :1;
        unsigned int :3;
        unsigned int SID0 :1;
        unsigned int SID1 :1;
        unsigned int SID2 :1;
    };
    struct {
        unsigned int RXM0EID16 :1;
        unsigned int RXM0EID17 :1;
        unsigned int :1;
        unsigned int RXM0EXIDM :1;
        unsigned int :1;
        unsigned int RXM0SID0 :1;
        unsigned int RXM0SID1 :1;
        unsigned int RXM0SID2 :1;
    };
} RXM0SIDLbits_t;
extern volatile RXM0SIDLbits_t RXM0SIDLbits __attribute__((address(0xEF9)));
# 19292 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXM0EIDH __attribute__((address(0xEFA)));

__asm("RXM0EIDH equ 0EFAh");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID8 :1;
        unsigned int EID9 :1;
        unsigned int EID10 :1;
        unsigned int EID11 :1;
        unsigned int EID12 :1;
        unsigned int EID13 :1;
        unsigned int EID14 :1;
        unsigned int EID15 :1;
    };
    struct {
        unsigned int RXM0EID8 :1;
        unsigned int RXM0EID9 :1;
        unsigned int RXM0EID10 :1;
        unsigned int RXM0EID11 :1;
        unsigned int RXM0EID12 :1;
        unsigned int RXM0EID13 :1;
        unsigned int RXM0EID14 :1;
        unsigned int RXM0EID15 :1;
    };
} RXM0EIDHbits_t;
extern volatile RXM0EIDHbits_t RXM0EIDHbits __attribute__((address(0xEFA)));
# 19412 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXM0EIDL __attribute__((address(0xEFB)));

__asm("RXM0EIDL equ 0EFBh");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID0 :1;
        unsigned int EID1 :1;
        unsigned int EID2 :1;
        unsigned int EID3 :1;
        unsigned int EID4 :1;
        unsigned int EID5 :1;
        unsigned int EID6 :1;
        unsigned int EID7 :1;
    };
    struct {
        unsigned int RXM0EID0 :1;
        unsigned int RXM0EID1 :1;
        unsigned int RXM0EID2 :1;
        unsigned int RXM0EID3 :1;
        unsigned int RXM0EID4 :1;
        unsigned int RXM0EID5 :1;
        unsigned int RXM0EID6 :1;
        unsigned int RXM0EID7 :1;
    };
} RXM0EIDLbits_t;
extern volatile RXM0EIDLbits_t RXM0EIDLbits __attribute__((address(0xEFB)));
# 19532 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXM1SIDH __attribute__((address(0xEFC)));

__asm("RXM1SIDH equ 0EFCh");


typedef union {
    struct {
        unsigned int SID :8;
    };
    struct {
        unsigned int SID3 :1;
        unsigned int SID4 :1;
        unsigned int SID5 :1;
        unsigned int SID6 :1;
        unsigned int SID7 :1;
        unsigned int SID8 :1;
        unsigned int SID9 :1;
        unsigned int SID10 :1;
    };
    struct {
        unsigned int RXM1SID3 :1;
        unsigned int RXM1SID4 :1;
        unsigned int RXM1SID5 :1;
        unsigned int RXM1SID6 :1;
        unsigned int RXM1SID7 :1;
        unsigned int RXM1SID8 :1;
        unsigned int RXM1SID9 :1;
        unsigned int RXM1SID10 :1;
    };
} RXM1SIDHbits_t;
extern volatile RXM1SIDHbits_t RXM1SIDHbits __attribute__((address(0xEFC)));
# 19652 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXM1SIDL __attribute__((address(0xEFD)));

__asm("RXM1SIDL equ 0EFDh");


typedef union {
    struct {
        unsigned int EID :2;
        unsigned int :1;
        unsigned int EXIDEN :1;
        unsigned int :1;
        unsigned int SID :3;
    };
    struct {
        unsigned int EID16 :1;
        unsigned int EID17 :1;
        unsigned int :3;
        unsigned int SID0 :1;
        unsigned int SID1 :1;
        unsigned int SID2 :1;
    };
    struct {
        unsigned int RXM1EID16 :1;
        unsigned int RXM1EID17 :1;
        unsigned int :1;
        unsigned int RXM1EXIDEN :1;
        unsigned int :1;
        unsigned int RXM1SID0 :1;
        unsigned int RXM1SID1 :1;
        unsigned int RXM1SID2 :1;
    };
} RXM1SIDLbits_t;
extern volatile RXM1SIDLbits_t RXM1SIDLbits __attribute__((address(0xEFD)));
# 19759 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXM1EIDH __attribute__((address(0xEFE)));

__asm("RXM1EIDH equ 0EFEh");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID8 :1;
        unsigned int EID9 :1;
        unsigned int EID10 :1;
        unsigned int EID11 :1;
        unsigned int EID12 :1;
        unsigned int EID13 :1;
        unsigned int EID14 :1;
        unsigned int EID15 :1;
    };
    struct {
        unsigned int RXM1EID8 :1;
        unsigned int RXM1EID9 :1;
        unsigned int RXM1EID10 :1;
        unsigned int RXM1EID11 :1;
        unsigned int RXM1EID12 :1;
        unsigned int RXM1EID13 :1;
        unsigned int RXM1EID14 :1;
        unsigned int RXM1EID15 :1;
    };
} RXM1EIDHbits_t;
extern volatile RXM1EIDHbits_t RXM1EIDHbits __attribute__((address(0xEFE)));
# 19879 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXM1EIDL __attribute__((address(0xEFF)));

__asm("RXM1EIDL equ 0EFFh");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID0 :1;
        unsigned int EID1 :1;
        unsigned int EID2 :1;
        unsigned int EID3 :1;
        unsigned int EID4 :1;
        unsigned int EID5 :1;
        unsigned int EID6 :1;
        unsigned int EID7 :1;
    };
    struct {
        unsigned int RXM1EID0 :1;
        unsigned int RXM1EID1 :1;
        unsigned int RXM1EID2 :1;
        unsigned int RXM1EID3 :1;
        unsigned int RXM1EID4 :1;
        unsigned int RXM1EID5 :1;
        unsigned int RXM1EID6 :1;
        unsigned int RXM1EID7 :1;
    };
} RXM1EIDLbits_t;
extern volatile RXM1EIDLbits_t RXM1EIDLbits __attribute__((address(0xEFF)));
# 19999 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB2CON __attribute__((address(0xF00)));

__asm("TXB2CON equ 0F00h");


typedef union {
    struct {
        unsigned int TXPRI :2;
        unsigned int :1;
        unsigned int TXREQ :1;
        unsigned int TXERR :1;
        unsigned int TXLARB :1;
        unsigned int TXABT :1;
        unsigned int TXBIF :1;
    };
    struct {
        unsigned int TXPRI0 :1;
        unsigned int TXPRI1 :1;
    };
    struct {
        unsigned int TXB2PRI0 :1;
        unsigned int TXB2PRI1 :1;
        unsigned int :1;
        unsigned int TXB2REQ :1;
        unsigned int TXB2ERR :1;
        unsigned int TXB2LARB :1;
        unsigned int TXB2ABT :1;
        unsigned int TX2IF :1;
    };
} TXB2CONbits_t;
extern volatile TXB2CONbits_t TXB2CONbits __attribute__((address(0xF00)));
# 20109 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB2SIDH __attribute__((address(0xF01)));

__asm("TXB2SIDH equ 0F01h");


typedef union {
    struct {
        unsigned int SID :8;
    };
    struct {
        unsigned int SID3 :1;
        unsigned int SID4 :1;
        unsigned int SID5 :1;
        unsigned int SID6 :1;
        unsigned int SID7 :1;
        unsigned int SID8 :1;
        unsigned int SID9 :1;
        unsigned int SID10 :1;
    };
    struct {
        unsigned int TXB2SID3 :1;
        unsigned int TXB2SID4 :1;
        unsigned int TXB2SID5 :1;
        unsigned int TXB2SID6 :1;
        unsigned int TXB2SID7 :1;
        unsigned int TXB2SID8 :1;
        unsigned int TXB2SID9 :1;
        unsigned int TXB2SID10 :1;
    };
} TXB2SIDHbits_t;
extern volatile TXB2SIDHbits_t TXB2SIDHbits __attribute__((address(0xF01)));
# 20229 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB2SIDL __attribute__((address(0xF02)));

__asm("TXB2SIDL equ 0F02h");


typedef union {
    struct {
        unsigned int EID :2;
        unsigned int :1;
        unsigned int EXIDE :1;
        unsigned int SRR :1;
        unsigned int SID :3;
    };
    struct {
        unsigned int EID16 :1;
        unsigned int EID17 :1;
        unsigned int :3;
        unsigned int SID0 :1;
        unsigned int SID1 :1;
        unsigned int SID2 :1;
    };
    struct {
        unsigned int TXB2EID16 :1;
        unsigned int TXB2EID17 :1;
        unsigned int :1;
        unsigned int TXB2EXIDE :1;
        unsigned int :1;
        unsigned int TXB2SID0 :1;
        unsigned int TXB2SID1 :1;
        unsigned int TXB2SID2 :1;
    };
} TXB2SIDLbits_t;
extern volatile TXB2SIDLbits_t TXB2SIDLbits __attribute__((address(0xF02)));
# 20341 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB2EIDH __attribute__((address(0xF03)));

__asm("TXB2EIDH equ 0F03h");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID8 :1;
        unsigned int EID9 :1;
        unsigned int EID10 :1;
        unsigned int EID11 :1;
        unsigned int EID12 :1;
        unsigned int EID13 :1;
        unsigned int EID14 :1;
        unsigned int EID15 :1;
    };
    struct {
        unsigned int TXB2EID8 :1;
        unsigned int TXB2EID9 :1;
        unsigned int TXB2EID10 :1;
        unsigned int TXB2EID11 :1;
        unsigned int TXB2EID12 :1;
        unsigned int TXB2EID13 :1;
        unsigned int TXB2EID14 :1;
        unsigned int TXB2EID15 :1;
    };
} TXB2EIDHbits_t;
extern volatile TXB2EIDHbits_t TXB2EIDHbits __attribute__((address(0xF03)));
# 20461 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB2EIDL __attribute__((address(0xF04)));

__asm("TXB2EIDL equ 0F04h");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID0 :1;
        unsigned int EID1 :1;
        unsigned int EID2 :1;
        unsigned int EID3 :1;
        unsigned int EID4 :1;
        unsigned int EID5 :1;
        unsigned int EID6 :1;
        unsigned int EID7 :1;
    };
    struct {
        unsigned int TXB2EID0 :1;
        unsigned int TXB2EID1 :1;
        unsigned int TXB2EID2 :1;
        unsigned int TXB2EID3 :1;
        unsigned int TXB2EID4 :1;
        unsigned int TXB2EID5 :1;
        unsigned int TXB2EID6 :1;
        unsigned int TXB2EID7 :1;
    };
} TXB2EIDLbits_t;
extern volatile TXB2EIDLbits_t TXB2EIDLbits __attribute__((address(0xF04)));
# 20581 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB2DLC __attribute__((address(0xF05)));

__asm("TXB2DLC equ 0F05h");


typedef union {
    struct {
        unsigned int DLC :4;
        unsigned int :2;
        unsigned int TXRTR :1;
    };
    struct {
        unsigned int DLC0 :1;
        unsigned int DLC1 :1;
        unsigned int DLC2 :1;
        unsigned int DLC3 :1;
    };
    struct {
        unsigned int TXB2DLC0 :1;
        unsigned int TXB2DLC1 :1;
        unsigned int TXB2DLC2 :1;
        unsigned int TXB2DLC3 :1;
        unsigned int :2;
        unsigned int TXB2RTR :1;
    };
} TXB2DLCbits_t;
extern volatile TXB2DLCbits_t TXB2DLCbits __attribute__((address(0xF05)));
# 20667 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB2D0 __attribute__((address(0xF06)));

__asm("TXB2D0 equ 0F06h");


typedef union {
    struct {
        unsigned int TXB2D0 :8;
    };
    struct {
        unsigned int TXB2D00 :1;
        unsigned int TXB2D01 :1;
        unsigned int TXB2D02 :1;
        unsigned int TXB2D03 :1;
        unsigned int TXB2D04 :1;
        unsigned int TXB2D05 :1;
        unsigned int TXB2D06 :1;
        unsigned int TXB2D07 :1;
    };
} TXB2D0bits_t;
extern volatile TXB2D0bits_t TXB2D0bits __attribute__((address(0xF06)));
# 20737 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB2D1 __attribute__((address(0xF07)));

__asm("TXB2D1 equ 0F07h");


typedef union {
    struct {
        unsigned int TXB2D1 :8;
    };
    struct {
        unsigned int TXB2D10 :1;
        unsigned int TXB2D11 :1;
        unsigned int TXB2D12 :1;
        unsigned int TXB2D13 :1;
        unsigned int TXB2D14 :1;
        unsigned int TXB2D15 :1;
        unsigned int TXB2D16 :1;
        unsigned int TXB2D17 :1;
    };
} TXB2D1bits_t;
extern volatile TXB2D1bits_t TXB2D1bits __attribute__((address(0xF07)));
# 20807 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB2D2 __attribute__((address(0xF08)));

__asm("TXB2D2 equ 0F08h");


typedef union {
    struct {
        unsigned int TXB2D2 :8;
    };
    struct {
        unsigned int TXB2D20 :1;
        unsigned int TXB2D21 :1;
        unsigned int TXB2D22 :1;
        unsigned int TXB2D23 :1;
        unsigned int TXB2D24 :1;
        unsigned int TXB2D25 :1;
        unsigned int TXB2D26 :1;
        unsigned int TXB2D27 :1;
    };
} TXB2D2bits_t;
extern volatile TXB2D2bits_t TXB2D2bits __attribute__((address(0xF08)));
# 20877 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB2D3 __attribute__((address(0xF09)));

__asm("TXB2D3 equ 0F09h");


typedef union {
    struct {
        unsigned int TXB2D3 :8;
    };
    struct {
        unsigned int TXB2D30 :1;
        unsigned int TXB2D31 :1;
        unsigned int TXB2D32 :1;
        unsigned int TXB2D33 :1;
        unsigned int TXB2D34 :1;
        unsigned int TXB2D35 :1;
        unsigned int TXB2D36 :1;
        unsigned int TXB2D37 :1;
    };
} TXB2D3bits_t;
extern volatile TXB2D3bits_t TXB2D3bits __attribute__((address(0xF09)));
# 20947 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB2D4 __attribute__((address(0xF0A)));

__asm("TXB2D4 equ 0F0Ah");


typedef union {
    struct {
        unsigned int TXB2D4 :8;
    };
    struct {
        unsigned int TXB2D40 :1;
        unsigned int TXB2D41 :1;
        unsigned int TXB2D42 :1;
        unsigned int TXB2D43 :1;
        unsigned int TXB2D44 :1;
        unsigned int TXB2D45 :1;
        unsigned int TXB2D46 :1;
        unsigned int TXB2D47 :1;
    };
} TXB2D4bits_t;
extern volatile TXB2D4bits_t TXB2D4bits __attribute__((address(0xF0A)));
# 21017 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB2D5 __attribute__((address(0xF0B)));

__asm("TXB2D5 equ 0F0Bh");


typedef union {
    struct {
        unsigned int TXB2D5 :8;
    };
    struct {
        unsigned int TXB2D50 :1;
        unsigned int TXB2D51 :1;
        unsigned int TXB2D52 :1;
        unsigned int TXB2D53 :1;
        unsigned int TXB2D54 :1;
        unsigned int TXB2D55 :1;
        unsigned int TXB2D56 :1;
        unsigned int TXB2D57 :1;
    };
} TXB2D5bits_t;
extern volatile TXB2D5bits_t TXB2D5bits __attribute__((address(0xF0B)));
# 21087 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB2D6 __attribute__((address(0xF0C)));

__asm("TXB2D6 equ 0F0Ch");


typedef union {
    struct {
        unsigned int TXB2D6 :8;
    };
    struct {
        unsigned int TXB2D60 :1;
        unsigned int TXB2D61 :1;
        unsigned int TXB2D62 :1;
        unsigned int TXB2D63 :1;
        unsigned int TXB2D64 :1;
        unsigned int TXB2D65 :1;
        unsigned int TXB2D66 :1;
        unsigned int TXB2D67 :1;
    };
} TXB2D6bits_t;
extern volatile TXB2D6bits_t TXB2D6bits __attribute__((address(0xF0C)));
# 21157 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB2D7 __attribute__((address(0xF0D)));

__asm("TXB2D7 equ 0F0Dh");


typedef union {
    struct {
        unsigned int TXB2D7 :8;
    };
    struct {
        unsigned int TXB2D70 :1;
        unsigned int TXB2D71 :1;
        unsigned int TXB2D72 :1;
        unsigned int TXB2D73 :1;
        unsigned int TXB2D74 :1;
        unsigned int TXB2D75 :1;
        unsigned int TXB2D76 :1;
        unsigned int TXB2D77 :1;
    };
} TXB2D7bits_t;
extern volatile TXB2D7bits_t TXB2D7bits __attribute__((address(0xF0D)));
# 21227 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CANSTAT_RO3 __attribute__((address(0xF0E)));

__asm("CANSTAT_RO3 equ 0F0Eh");


typedef union {
    struct {
        unsigned int EICODE0 :1;
        unsigned int EICODE1_ICODE0 :1;
        unsigned int EICODE2_ICODE1 :1;
        unsigned int EICODE3_ICODE2 :1;
        unsigned int EICODE4 :1;
        unsigned int OPMODE :3;
    };
    struct {
        unsigned int :1;
        unsigned int EICODE1 :1;
        unsigned int EICODE2 :1;
        unsigned int EICODE3 :1;
        unsigned int :1;
        unsigned int OPMODE0 :1;
        unsigned int OPMODE1 :1;
        unsigned int OPMODE2 :1;
    };
    struct {
        unsigned int :1;
        unsigned int ICODE0 :1;
        unsigned int ICODE1 :1;
        unsigned int ICODE2 :1;
    };
} CANSTAT_RO3bits_t;
extern volatile CANSTAT_RO3bits_t CANSTAT_RO3bits __attribute__((address(0xF0E)));
# 21338 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CANCON_RO3 __attribute__((address(0xF0F)));

__asm("CANCON_RO3 equ 0F0Fh");


typedef union {
    struct {
        unsigned int FP0 :1;
        unsigned int WIN0_FP1 :1;
        unsigned int WIN1_FP2 :1;
        unsigned int WIN2_FP3 :1;
        unsigned int ABAT :1;
        unsigned int REQOP :3;
    };
    struct {
        unsigned int :1;
        unsigned int WIN0 :1;
        unsigned int WIN1 :1;
        unsigned int WIN2 :1;
    };
    struct {
        unsigned int :1;
        unsigned int FP1 :1;
        unsigned int FP2 :1;
        unsigned int FP3 :1;
    };
} CANCON_RO3bits_t;
extern volatile CANCON_RO3bits_t CANCON_RO3bits __attribute__((address(0xF0F)));
# 21430 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB1CON __attribute__((address(0xF10)));

__asm("TXB1CON equ 0F10h");


typedef union {
    struct {
        unsigned int TXPRI :2;
        unsigned int :1;
        unsigned int TXREQ :1;
        unsigned int TXERR :1;
        unsigned int TXLARB :1;
        unsigned int TXABT :1;
        unsigned int TXBIF :1;
    };
    struct {
        unsigned int TXPRI0 :1;
        unsigned int TXPRI1 :1;
    };
    struct {
        unsigned int TXB1PRI0 :1;
        unsigned int TXB1PRI1 :1;
        unsigned int :1;
        unsigned int TXB1REQ :1;
        unsigned int TXB1ERR :1;
        unsigned int TXB1LARB :1;
        unsigned int TXB1ABT :1;
        unsigned int TX1IF :1;
    };
} TXB1CONbits_t;
extern volatile TXB1CONbits_t TXB1CONbits __attribute__((address(0xF10)));
# 21540 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB1SIDH __attribute__((address(0xF11)));

__asm("TXB1SIDH equ 0F11h");


typedef union {
    struct {
        unsigned int SID :8;
    };
    struct {
        unsigned int SID3 :1;
        unsigned int SID4 :1;
        unsigned int SID5 :1;
        unsigned int SID6 :1;
        unsigned int SID7 :1;
        unsigned int SID8 :1;
        unsigned int SID9 :1;
        unsigned int SID10 :1;
    };
    struct {
        unsigned int TXB1SID3 :1;
        unsigned int TXB1SID4 :1;
        unsigned int TXB1SID5 :1;
        unsigned int TXB1SID6 :1;
        unsigned int TXB1SID7 :1;
        unsigned int TXB1SID8 :1;
        unsigned int TXB1SID9 :1;
        unsigned int TXB1SID10 :1;
    };
} TXB1SIDHbits_t;
extern volatile TXB1SIDHbits_t TXB1SIDHbits __attribute__((address(0xF11)));
# 21660 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB1SIDL __attribute__((address(0xF12)));

__asm("TXB1SIDL equ 0F12h");


typedef union {
    struct {
        unsigned int EID :2;
        unsigned int :1;
        unsigned int EXIDE :1;
        unsigned int SRR :1;
        unsigned int SID :3;
    };
    struct {
        unsigned int EID16 :1;
        unsigned int EID17 :1;
        unsigned int :3;
        unsigned int SID0 :1;
        unsigned int SID1 :1;
        unsigned int SID2 :1;
    };
    struct {
        unsigned int TXB1EID16 :1;
        unsigned int TXB1EID17 :1;
        unsigned int :1;
        unsigned int TXB1EXIDE :1;
        unsigned int :1;
        unsigned int TXB1SID0 :1;
        unsigned int TXB1SID1 :1;
        unsigned int TXB1SID2 :1;
    };
} TXB1SIDLbits_t;
extern volatile TXB1SIDLbits_t TXB1SIDLbits __attribute__((address(0xF12)));
# 21772 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB1EIDH __attribute__((address(0xF13)));

__asm("TXB1EIDH equ 0F13h");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID8 :1;
        unsigned int EID9 :1;
        unsigned int EID10 :1;
        unsigned int EID11 :1;
        unsigned int EID12 :1;
        unsigned int EID13 :1;
        unsigned int EID14 :1;
        unsigned int EID15 :1;
    };
    struct {
        unsigned int TXB1EID8 :1;
        unsigned int TXB1EID9 :1;
        unsigned int TXB1EID10 :1;
        unsigned int TXB1EID11 :1;
        unsigned int TXB1EID12 :1;
        unsigned int TXB1EID13 :1;
        unsigned int TXB1EID14 :1;
        unsigned int TXB1EID15 :1;
    };
} TXB1EIDHbits_t;
extern volatile TXB1EIDHbits_t TXB1EIDHbits __attribute__((address(0xF13)));
# 21892 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB1EIDL __attribute__((address(0xF14)));

__asm("TXB1EIDL equ 0F14h");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID0 :1;
        unsigned int EID1 :1;
        unsigned int EID2 :1;
        unsigned int EID3 :1;
        unsigned int EID4 :1;
        unsigned int EID5 :1;
        unsigned int EID6 :1;
        unsigned int EID7 :1;
    };
    struct {
        unsigned int TXB1EID0 :1;
        unsigned int TXB1EID1 :1;
        unsigned int TXB1EID2 :1;
        unsigned int TXB1EID3 :1;
        unsigned int TXB1EID4 :1;
        unsigned int TXB1EID5 :1;
        unsigned int TXB1EID6 :1;
        unsigned int TXB1EID7 :1;
    };
} TXB1EIDLbits_t;
extern volatile TXB1EIDLbits_t TXB1EIDLbits __attribute__((address(0xF14)));
# 22012 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB1DLC __attribute__((address(0xF15)));

__asm("TXB1DLC equ 0F15h");


typedef union {
    struct {
        unsigned int DLC :4;
        unsigned int :2;
        unsigned int TXRTR :1;
    };
    struct {
        unsigned int DLC0 :1;
        unsigned int DLC1 :1;
        unsigned int DLC2 :1;
        unsigned int DLC3 :1;
    };
    struct {
        unsigned int TXB1DLC0 :1;
        unsigned int TXB1DLC1 :1;
        unsigned int TXB1DLC2 :1;
        unsigned int TXB1DLC3 :1;
        unsigned int :2;
        unsigned int TXB1RTR :1;
    };
} TXB1DLCbits_t;
extern volatile TXB1DLCbits_t TXB1DLCbits __attribute__((address(0xF15)));
# 22098 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB1D0 __attribute__((address(0xF16)));

__asm("TXB1D0 equ 0F16h");


typedef union {
    struct {
        unsigned int TXB1D0 :8;
    };
    struct {
        unsigned int TXB1D00 :1;
        unsigned int TXB1D01 :1;
        unsigned int TXB1D02 :1;
        unsigned int TXB1D03 :1;
        unsigned int TXB1D04 :1;
        unsigned int TXB1D05 :1;
        unsigned int TXB1D06 :1;
        unsigned int TXB1D07 :1;
    };
} TXB1D0bits_t;
extern volatile TXB1D0bits_t TXB1D0bits __attribute__((address(0xF16)));
# 22168 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB1D1 __attribute__((address(0xF17)));

__asm("TXB1D1 equ 0F17h");


typedef union {
    struct {
        unsigned int TXB1D1 :8;
    };
    struct {
        unsigned int TXB1D10 :1;
        unsigned int TXB1D11 :1;
        unsigned int TXB1D12 :1;
        unsigned int TXB1D13 :1;
        unsigned int TXB1D14 :1;
        unsigned int TXB1D15 :1;
        unsigned int TXB1D16 :1;
        unsigned int TXB1D17 :1;
    };
} TXB1D1bits_t;
extern volatile TXB1D1bits_t TXB1D1bits __attribute__((address(0xF17)));
# 22238 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB1D2 __attribute__((address(0xF18)));

__asm("TXB1D2 equ 0F18h");


typedef union {
    struct {
        unsigned int TXB1D2 :8;
    };
    struct {
        unsigned int TXB1D20 :1;
        unsigned int TXB1D21 :1;
        unsigned int TXB1D22 :1;
        unsigned int TXB1D23 :1;
        unsigned int TXB1D24 :1;
        unsigned int TXB1D25 :1;
        unsigned int TXB1D26 :1;
        unsigned int TXB1D27 :1;
    };
} TXB1D2bits_t;
extern volatile TXB1D2bits_t TXB1D2bits __attribute__((address(0xF18)));
# 22308 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB1D3 __attribute__((address(0xF19)));

__asm("TXB1D3 equ 0F19h");


typedef union {
    struct {
        unsigned int TXB1D3 :8;
    };
    struct {
        unsigned int TXB1D30 :1;
        unsigned int TXB1D31 :1;
        unsigned int TXB1D32 :1;
        unsigned int TXB1D33 :1;
        unsigned int TXB1D34 :1;
        unsigned int TXB1D35 :1;
        unsigned int TXB1D36 :1;
        unsigned int TXB1D37 :1;
    };
} TXB1D3bits_t;
extern volatile TXB1D3bits_t TXB1D3bits __attribute__((address(0xF19)));
# 22378 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB1D4 __attribute__((address(0xF1A)));

__asm("TXB1D4 equ 0F1Ah");


typedef union {
    struct {
        unsigned int TXB1D4 :8;
    };
    struct {
        unsigned int TXB1D40 :1;
        unsigned int TXB1D41 :1;
        unsigned int TXB1D42 :1;
        unsigned int TXB1D43 :1;
        unsigned int TXB1D44 :1;
        unsigned int TXB1D45 :1;
        unsigned int TXB1D46 :1;
        unsigned int TXB1D47 :1;
    };
} TXB1D4bits_t;
extern volatile TXB1D4bits_t TXB1D4bits __attribute__((address(0xF1A)));
# 22448 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB1D5 __attribute__((address(0xF1B)));

__asm("TXB1D5 equ 0F1Bh");


typedef union {
    struct {
        unsigned int TXB1D5 :8;
    };
    struct {
        unsigned int TXB1D50 :1;
        unsigned int TXB1D51 :1;
        unsigned int TXB1D52 :1;
        unsigned int TXB1D53 :1;
        unsigned int TXB1D54 :1;
        unsigned int TXB1D55 :1;
        unsigned int TXB1D56 :1;
        unsigned int TXB1D57 :1;
    };
} TXB1D5bits_t;
extern volatile TXB1D5bits_t TXB1D5bits __attribute__((address(0xF1B)));
# 22518 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB1D6 __attribute__((address(0xF1C)));

__asm("TXB1D6 equ 0F1Ch");


typedef union {
    struct {
        unsigned int TXB1D6 :8;
    };
    struct {
        unsigned int TXB1D60 :1;
        unsigned int TXB1D61 :1;
        unsigned int TXB1D62 :1;
        unsigned int TXB1D63 :1;
        unsigned int TXB1D64 :1;
        unsigned int TXB1D65 :1;
        unsigned int TXB1D66 :1;
        unsigned int TXB1D67 :1;
    };
} TXB1D6bits_t;
extern volatile TXB1D6bits_t TXB1D6bits __attribute__((address(0xF1C)));
# 22588 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB1D7 __attribute__((address(0xF1D)));

__asm("TXB1D7 equ 0F1Dh");


typedef union {
    struct {
        unsigned int TXB1D7 :8;
    };
    struct {
        unsigned int TXB1D70 :1;
        unsigned int TXB1D71 :1;
        unsigned int TXB1D72 :1;
        unsigned int TXB1D73 :1;
        unsigned int TXB1D74 :1;
        unsigned int TXB1D75 :1;
        unsigned int TXB1D76 :1;
        unsigned int TXB1D77 :1;
    };
} TXB1D7bits_t;
extern volatile TXB1D7bits_t TXB1D7bits __attribute__((address(0xF1D)));
# 22658 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CANSTAT_RO2 __attribute__((address(0xF1E)));

__asm("CANSTAT_RO2 equ 0F1Eh");


typedef union {
    struct {
        unsigned int EICODE0 :1;
        unsigned int EICODE1_ICODE0 :1;
        unsigned int EICODE2_ICODE1 :1;
        unsigned int EICODE3_ICODE2 :1;
        unsigned int EICODE4 :1;
        unsigned int OPMODE :3;
    };
    struct {
        unsigned int :1;
        unsigned int EICODE1 :1;
        unsigned int EICODE2 :1;
        unsigned int EICODE3 :1;
        unsigned int :1;
        unsigned int OPMODE0 :1;
        unsigned int OPMODE1 :1;
        unsigned int OPMODE2 :1;
    };
    struct {
        unsigned int :1;
        unsigned int ICODE20 :1;
        unsigned int ICODE21 :1;
        unsigned int ICODE2 :1;
    };
} CANSTAT_RO2bits_t;
extern volatile CANSTAT_RO2bits_t CANSTAT_RO2bits __attribute__((address(0xF1E)));
# 22769 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CANCON_RO2 __attribute__((address(0xF1F)));

__asm("CANCON_RO2 equ 0F1Fh");


typedef union {
    struct {
        unsigned int FP0 :1;
        unsigned int WIN0_FP1 :1;
        unsigned int WIN1_FP2 :1;
        unsigned int WIN2_FP3 :1;
        unsigned int ABAT :1;
        unsigned int REQOP :3;
    };
    struct {
        unsigned int :1;
        unsigned int WIN0 :1;
        unsigned int WIN1 :1;
        unsigned int WIN2 :1;
    };
    struct {
        unsigned int :1;
        unsigned int FP1 :1;
        unsigned int FP2 :1;
        unsigned int FP3 :1;
    };
} CANCON_RO2bits_t;
extern volatile CANCON_RO2bits_t CANCON_RO2bits __attribute__((address(0xF1F)));
# 22861 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB0CON __attribute__((address(0xF20)));

__asm("TXB0CON equ 0F20h");


typedef union {
    struct {
        unsigned int TXPRI :2;
        unsigned int :1;
        unsigned int TXREQ :1;
        unsigned int TXERR :1;
        unsigned int TXLARB :1;
        unsigned int TXABT :1;
        unsigned int TXBIF :1;
    };
    struct {
        unsigned int TXPRI0 :1;
        unsigned int TXPRI1 :1;
    };
    struct {
        unsigned int TXB0PRI0 :1;
        unsigned int TXB0PRI1 :1;
        unsigned int :1;
        unsigned int TXB0REQ :1;
        unsigned int TXB0ERR :1;
        unsigned int TXB0LARB :1;
        unsigned int TXB0ABT :1;
        unsigned int TX0IF :1;
    };
} TXB0CONbits_t;
extern volatile TXB0CONbits_t TXB0CONbits __attribute__((address(0xF20)));
# 22971 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB0SIDH __attribute__((address(0xF21)));

__asm("TXB0SIDH equ 0F21h");


typedef union {
    struct {
        unsigned int SID :8;
    };
    struct {
        unsigned int SID3 :1;
        unsigned int SID4 :1;
        unsigned int SID5 :1;
        unsigned int SID6 :1;
        unsigned int SID7 :1;
        unsigned int SID8 :1;
        unsigned int SID9 :1;
        unsigned int SID10 :1;
    };
    struct {
        unsigned int TXB0SID3 :1;
        unsigned int TXB0SID4 :1;
        unsigned int TXB0SID5 :1;
        unsigned int TXB0SID6 :1;
        unsigned int TXB0SID7 :1;
        unsigned int TXB0SID8 :1;
        unsigned int TXB0SID9 :1;
        unsigned int TXB0SID10 :1;
    };
} TXB0SIDHbits_t;
extern volatile TXB0SIDHbits_t TXB0SIDHbits __attribute__((address(0xF21)));
# 23091 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB0SIDL __attribute__((address(0xF22)));

__asm("TXB0SIDL equ 0F22h");


typedef union {
    struct {
        unsigned int EID :2;
        unsigned int :1;
        unsigned int EXIDE :1;
        unsigned int SRR :1;
        unsigned int SID :3;
    };
    struct {
        unsigned int EID16 :1;
        unsigned int EID17 :1;
        unsigned int :3;
        unsigned int SID0 :1;
        unsigned int SID1 :1;
        unsigned int SID2 :1;
    };
    struct {
        unsigned int TXB0EID16 :1;
        unsigned int TXB0EID17 :1;
        unsigned int :1;
        unsigned int TXB0EXIDE :1;
        unsigned int :1;
        unsigned int TXB0SID0 :1;
        unsigned int TXB0SID1 :1;
        unsigned int TXB0SID2 :1;
    };
} TXB0SIDLbits_t;
extern volatile TXB0SIDLbits_t TXB0SIDLbits __attribute__((address(0xF22)));
# 23203 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB0EIDH __attribute__((address(0xF23)));

__asm("TXB0EIDH equ 0F23h");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID8 :1;
        unsigned int EID9 :1;
        unsigned int EID10 :1;
        unsigned int EID11 :1;
        unsigned int EID12 :1;
        unsigned int EID13 :1;
        unsigned int EID14 :1;
        unsigned int EID15 :1;
    };
    struct {
        unsigned int TXB0EID8 :1;
        unsigned int TXB0EID9 :1;
        unsigned int TXB0EID10 :1;
        unsigned int TXB0EID11 :1;
        unsigned int TXB0EID12 :1;
        unsigned int TXB0EID13 :1;
        unsigned int TXB0EID14 :1;
        unsigned int TXB0EID15 :1;
    };
} TXB0EIDHbits_t;
extern volatile TXB0EIDHbits_t TXB0EIDHbits __attribute__((address(0xF23)));
# 23323 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB0EIDL __attribute__((address(0xF24)));

__asm("TXB0EIDL equ 0F24h");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID0 :1;
        unsigned int EID1 :1;
        unsigned int EID2 :1;
        unsigned int EID3 :1;
        unsigned int EID4 :1;
        unsigned int EID5 :1;
        unsigned int EID6 :1;
        unsigned int EID7 :1;
    };
    struct {
        unsigned int TXB0EID0 :1;
        unsigned int TXB0EID1 :1;
        unsigned int TXB0EID2 :1;
        unsigned int TXB0EID3 :1;
        unsigned int TXB0EID4 :1;
        unsigned int TXB0EID5 :1;
        unsigned int TXB0EID6 :1;
        unsigned int TXB0EID7 :1;
    };
} TXB0EIDLbits_t;
extern volatile TXB0EIDLbits_t TXB0EIDLbits __attribute__((address(0xF24)));
# 23443 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB0DLC __attribute__((address(0xF25)));

__asm("TXB0DLC equ 0F25h");


typedef union {
    struct {
        unsigned int DLC :4;
        unsigned int :2;
        unsigned int TXRTR :1;
    };
    struct {
        unsigned int DLC0 :1;
        unsigned int DLC1 :1;
        unsigned int DLC2 :1;
        unsigned int DLC3 :1;
    };
    struct {
        unsigned int TXB0DLC0 :1;
        unsigned int TXB0DLC1 :1;
        unsigned int TXB0DLC2 :1;
        unsigned int TXB0DLC3 :1;
        unsigned int :2;
        unsigned int TXB0RTR :1;
    };
} TXB0DLCbits_t;
extern volatile TXB0DLCbits_t TXB0DLCbits __attribute__((address(0xF25)));
# 23529 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB0D0 __attribute__((address(0xF26)));

__asm("TXB0D0 equ 0F26h");


typedef union {
    struct {
        unsigned int TXB0D0 :8;
    };
    struct {
        unsigned int TXB0D00 :1;
        unsigned int TXB0D01 :1;
        unsigned int TXB0D02 :1;
        unsigned int TXB0D03 :1;
        unsigned int TXB0D04 :1;
        unsigned int TXB0D05 :1;
        unsigned int TXB0D06 :1;
        unsigned int TXB0D07 :1;
    };
} TXB0D0bits_t;
extern volatile TXB0D0bits_t TXB0D0bits __attribute__((address(0xF26)));
# 23599 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB0D1 __attribute__((address(0xF27)));

__asm("TXB0D1 equ 0F27h");


typedef union {
    struct {
        unsigned int TXB0D1 :8;
    };
    struct {
        unsigned int TXB0D10 :1;
        unsigned int TXB0D11 :1;
        unsigned int TXB0D12 :1;
        unsigned int TXB0D13 :1;
        unsigned int TXB0D14 :1;
        unsigned int TXB0D15 :1;
        unsigned int TXB0D16 :1;
        unsigned int TXB0D17 :1;
    };
} TXB0D1bits_t;
extern volatile TXB0D1bits_t TXB0D1bits __attribute__((address(0xF27)));
# 23669 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB0D2 __attribute__((address(0xF28)));

__asm("TXB0D2 equ 0F28h");


typedef union {
    struct {
        unsigned int TXB0D2 :8;
    };
    struct {
        unsigned int TXB0D20 :1;
        unsigned int TXB0D21 :1;
        unsigned int TXB0D22 :1;
        unsigned int TXB0D23 :1;
        unsigned int TXB0D24 :1;
        unsigned int TXB0D25 :1;
        unsigned int TXB0D26 :1;
        unsigned int TXB0D27 :1;
    };
} TXB0D2bits_t;
extern volatile TXB0D2bits_t TXB0D2bits __attribute__((address(0xF28)));
# 23739 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB0D3 __attribute__((address(0xF29)));

__asm("TXB0D3 equ 0F29h");


typedef union {
    struct {
        unsigned int TXB0D3 :8;
    };
    struct {
        unsigned int TXB0D30 :1;
        unsigned int TXB0D31 :1;
        unsigned int TXB0D32 :1;
        unsigned int TXB0D33 :1;
        unsigned int TXB0D34 :1;
        unsigned int TXB0D35 :1;
        unsigned int TXB0D36 :1;
        unsigned int TXB0D37 :1;
    };
} TXB0D3bits_t;
extern volatile TXB0D3bits_t TXB0D3bits __attribute__((address(0xF29)));
# 23809 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB0D4 __attribute__((address(0xF2A)));

__asm("TXB0D4 equ 0F2Ah");


typedef union {
    struct {
        unsigned int TXB0D4 :8;
    };
    struct {
        unsigned int TXB0D40 :1;
        unsigned int TXB0D41 :1;
        unsigned int TXB0D42 :1;
        unsigned int TXB0D43 :1;
        unsigned int TXB0D44 :1;
        unsigned int TXB0D45 :1;
        unsigned int TXB0D46 :1;
        unsigned int TXB0D47 :1;
    };
} TXB0D4bits_t;
extern volatile TXB0D4bits_t TXB0D4bits __attribute__((address(0xF2A)));
# 23879 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB0D5 __attribute__((address(0xF2B)));

__asm("TXB0D5 equ 0F2Bh");


typedef union {
    struct {
        unsigned int TXB0D5 :8;
    };
    struct {
        unsigned int TXB0D50 :1;
        unsigned int TXB0D51 :1;
        unsigned int TXB0D52 :1;
        unsigned int TXB0D53 :1;
        unsigned int TXB0D54 :1;
        unsigned int TXB0D55 :1;
        unsigned int TXB0D56 :1;
        unsigned int TXB0D57 :1;
    };
} TXB0D5bits_t;
extern volatile TXB0D5bits_t TXB0D5bits __attribute__((address(0xF2B)));
# 23949 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB0D6 __attribute__((address(0xF2C)));

__asm("TXB0D6 equ 0F2Ch");


typedef union {
    struct {
        unsigned int TXB0D6 :8;
    };
    struct {
        unsigned int TXB0D60 :1;
        unsigned int TXB0D61 :1;
        unsigned int TXB0D62 :1;
        unsigned int TXB0D63 :1;
        unsigned int TXB0D64 :1;
        unsigned int TXB0D65 :1;
        unsigned int TXB0D66 :1;
        unsigned int TXB0D67 :1;
    };
} TXB0D6bits_t;
extern volatile TXB0D6bits_t TXB0D6bits __attribute__((address(0xF2C)));
# 24019 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXB0D7 __attribute__((address(0xF2D)));

__asm("TXB0D7 equ 0F2Dh");


typedef union {
    struct {
        unsigned int TXB0D7 :8;
    };
    struct {
        unsigned int TXB0D70 :1;
        unsigned int TXB0D71 :1;
        unsigned int TXB0D72 :1;
        unsigned int TXB0D73 :1;
        unsigned int TXB0D74 :1;
        unsigned int TXB0D75 :1;
        unsigned int TXB0D76 :1;
        unsigned int TXB0D77 :1;
    };
} TXB0D7bits_t;
extern volatile TXB0D7bits_t TXB0D7bits __attribute__((address(0xF2D)));
# 24089 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CANSTAT_RO1 __attribute__((address(0xF2E)));

__asm("CANSTAT_RO1 equ 0F2Eh");


typedef union {
    struct {
        unsigned int EICODE0 :1;
        unsigned int EICODE1_ICODE0 :1;
        unsigned int EICODE2_ICODE1 :1;
        unsigned int EICODE3_ICODE2 :1;
        unsigned int EICODE4 :1;
        unsigned int OPMODE :3;
    };
    struct {
        unsigned int :1;
        unsigned int EICODE1 :1;
        unsigned int EICODE2 :1;
        unsigned int EICODE3 :1;
        unsigned int :1;
        unsigned int OPMODE0 :1;
        unsigned int OPMODE1 :1;
        unsigned int OPMODE2 :1;
    };
    struct {
        unsigned int :1;
        unsigned int ICODE0 :1;
        unsigned int ICODE1 :1;
        unsigned int ICODE2 :1;
    };
} CANSTAT_RO1bits_t;
extern volatile CANSTAT_RO1bits_t CANSTAT_RO1bits __attribute__((address(0xF2E)));
# 24200 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CANCON_RO1 __attribute__((address(0xF2F)));

__asm("CANCON_RO1 equ 0F2Fh");


typedef union {
    struct {
        unsigned int FP0 :1;
        unsigned int WIN0_FP1 :1;
        unsigned int WIN1_FP2 :1;
        unsigned int WIN2_FP3 :1;
        unsigned int ABAT :1;
        unsigned int REQOP :3;
    };
    struct {
        unsigned int :1;
        unsigned int WIN0 :1;
        unsigned int WIN1 :1;
        unsigned int WIN2 :1;
    };
    struct {
        unsigned int :1;
        unsigned int FP1 :1;
        unsigned int FP2 :1;
        unsigned int FP3 :1;
    };
} CANCON_RO1bits_t;
extern volatile CANCON_RO1bits_t CANCON_RO1bits __attribute__((address(0xF2F)));
# 24292 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXB1CON __attribute__((address(0xF30)));

__asm("RXB1CON equ 0F30h");


typedef union {
    struct {
        unsigned int FILHIT0 :1;
        unsigned int FILHIT1 :1;
        unsigned int FILHIT2 :1;
        unsigned int RXRTRRO_FILHIT3 :1;
        unsigned int FILHIT4 :1;
        unsigned int RXM0_RTRRO :1;
        unsigned int RXM1 :1;
        unsigned int RXFUL :1;
    };
    struct {
        unsigned int :3;
        unsigned int RXRTRRO :1;
        unsigned int :1;
        unsigned int RXM0 :1;
    };
    struct {
        unsigned int :3;
        unsigned int FILHIT3 :1;
        unsigned int :1;
        unsigned int RTRRO :1;
    };
    struct {
        unsigned int RXB1FILHIT0 :1;
        unsigned int RXB1FILHIT1 :1;
        unsigned int RXB1FILHIT2 :1;
        unsigned int RXB1FILHIT3 :1;
        unsigned int RXB1FILHIT4 :1;
        unsigned int RXB1M0 :1;
        unsigned int RXB1M1 :1;
        unsigned int RXB1FUL :1;
    };
    struct {
        unsigned int :3;
        unsigned int RXB1RTRR0 :1;
        unsigned int :1;
        unsigned int RXB1RTRRO :1;
    };
} RXB1CONbits_t;
extern volatile RXB1CONbits_t RXB1CONbits __attribute__((address(0xF30)));
# 24452 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXB1SIDH __attribute__((address(0xF31)));

__asm("RXB1SIDH equ 0F31h");


typedef union {
    struct {
        unsigned int SID :8;
    };
    struct {
        unsigned int SID3 :1;
        unsigned int SID4 :1;
        unsigned int SID5 :1;
        unsigned int SID6 :1;
        unsigned int SID7 :1;
        unsigned int SID8 :1;
        unsigned int SID9 :1;
        unsigned int SID10 :1;
    };
    struct {
        unsigned int RXB1SID3 :1;
        unsigned int RXB1SID4 :1;
        unsigned int RXB1SID5 :1;
        unsigned int RXB1SID6 :1;
        unsigned int RXB1SID7 :1;
        unsigned int RXB1SID8 :1;
        unsigned int RXB1SID9 :1;
        unsigned int RXB1SID10 :1;
    };
} RXB1SIDHbits_t;
extern volatile RXB1SIDHbits_t RXB1SIDHbits __attribute__((address(0xF31)));
# 24572 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXB1SIDL __attribute__((address(0xF32)));

__asm("RXB1SIDL equ 0F32h");


typedef union {
    struct {
        unsigned int EID :2;
        unsigned int :1;
        unsigned int EXID :1;
        unsigned int SRR :1;
        unsigned int SID :3;
    };
    struct {
        unsigned int EDI16 :1;
        unsigned int EDI17 :1;
        unsigned int :3;
        unsigned int SID0 :1;
        unsigned int SID1 :1;
        unsigned int SID2 :1;
    };
    struct {
        unsigned int RXB1EID16 :1;
        unsigned int RXB1EID17 :1;
        unsigned int :1;
        unsigned int RXB1EXID :1;
        unsigned int RXB1SRR :1;
        unsigned int RXB1SID0 :1;
        unsigned int RXB1SID1 :1;
        unsigned int RXB1SID2 :1;
    };
} RXB1SIDLbits_t;
extern volatile RXB1SIDLbits_t RXB1SIDLbits __attribute__((address(0xF32)));
# 24689 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXB1EIDH __attribute__((address(0xF33)));

__asm("RXB1EIDH equ 0F33h");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID8 :1;
        unsigned int EID9 :1;
        unsigned int EID10 :1;
        unsigned int EID11 :1;
        unsigned int EID12 :1;
        unsigned int EID13 :1;
        unsigned int EID14 :1;
        unsigned int EID15 :1;
    };
    struct {
        unsigned int RXB1EID8 :1;
        unsigned int RXB1EID9 :1;
        unsigned int RXB1EID10 :1;
        unsigned int RXB1EID11 :1;
        unsigned int RXB1EID12 :1;
        unsigned int RXB1EID13 :1;
        unsigned int RXB1EID14 :1;
        unsigned int RXB1EID15 :1;
    };
} RXB1EIDHbits_t;
extern volatile RXB1EIDHbits_t RXB1EIDHbits __attribute__((address(0xF33)));
# 24809 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXB1EIDL __attribute__((address(0xF34)));

__asm("RXB1EIDL equ 0F34h");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID0 :1;
        unsigned int EID1 :1;
        unsigned int EID2 :1;
        unsigned int EID3 :1;
        unsigned int EID4 :1;
        unsigned int EID5 :1;
        unsigned int EID6 :1;
        unsigned int EID7 :1;
    };
    struct {
        unsigned int RXB1EID0 :1;
        unsigned int RXB1EID1 :1;
        unsigned int RXB1EID2 :1;
        unsigned int RXB1EID3 :1;
        unsigned int RXB1EID4 :1;
        unsigned int RXB1EID5 :1;
        unsigned int RXB1EID6 :1;
        unsigned int RXB1EID7 :1;
    };
} RXB1EIDLbits_t;
extern volatile RXB1EIDLbits_t RXB1EIDLbits __attribute__((address(0xF34)));
# 24929 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXB1DLC __attribute__((address(0xF35)));

__asm("RXB1DLC equ 0F35h");


typedef union {
    struct {
        unsigned int DLC :4;
        unsigned int RB :2;
        unsigned int RXRTR :1;
    };
    struct {
        unsigned int DLC0 :1;
        unsigned int DLC1 :1;
        unsigned int DLC2 :1;
        unsigned int DLC3 :1;
        unsigned int RB0 :1;
        unsigned int RB1 :1;
    };
    struct {
        unsigned int RXB1DLC0 :1;
        unsigned int RXB1DLC1 :1;
        unsigned int RXB1DLC2 :1;
        unsigned int RXB1DLC3 :1;
        unsigned int RXB1RB0 :1;
        unsigned int RXB1RB1 :1;
        unsigned int RXB1RTR :1;
    };
} RXB1DLCbits_t;
extern volatile RXB1DLCbits_t RXB1DLCbits __attribute__((address(0xF35)));
# 25043 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXB1D0 __attribute__((address(0xF36)));

__asm("RXB1D0 equ 0F36h");


typedef union {
    struct {
        unsigned int RXB1D0 :8;
    };
    struct {
        unsigned int RXB1D00 :1;
        unsigned int RXB1D01 :1;
        unsigned int RXB1D02 :1;
        unsigned int RXB1D03 :1;
        unsigned int RXB1D04 :1;
        unsigned int RXB1D05 :1;
        unsigned int RXB1D06 :1;
        unsigned int RXB1D07 :1;
    };
} RXB1D0bits_t;
extern volatile RXB1D0bits_t RXB1D0bits __attribute__((address(0xF36)));
# 25113 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXB1D1 __attribute__((address(0xF37)));

__asm("RXB1D1 equ 0F37h");


typedef union {
    struct {
        unsigned int RXB1D1 :8;
    };
    struct {
        unsigned int RXB1D10 :1;
        unsigned int RXB1D11 :1;
        unsigned int RXB1D12 :1;
        unsigned int RXB1D13 :1;
        unsigned int RXB1D14 :1;
        unsigned int RXB1D15 :1;
        unsigned int RXB1D16 :1;
        unsigned int RXB1D17 :1;
    };
} RXB1D1bits_t;
extern volatile RXB1D1bits_t RXB1D1bits __attribute__((address(0xF37)));
# 25183 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXB1D2 __attribute__((address(0xF38)));

__asm("RXB1D2 equ 0F38h");


typedef union {
    struct {
        unsigned int RXB1D2 :8;
    };
    struct {
        unsigned int RXB1D20 :1;
        unsigned int RXB1D21 :1;
        unsigned int RXB1D22 :1;
        unsigned int RXB1D23 :1;
        unsigned int RXB1D24 :1;
        unsigned int RXB1D25 :1;
        unsigned int RXB1D26 :1;
        unsigned int RXB1D27 :1;
    };
} RXB1D2bits_t;
extern volatile RXB1D2bits_t RXB1D2bits __attribute__((address(0xF38)));
# 25253 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXB1D3 __attribute__((address(0xF39)));

__asm("RXB1D3 equ 0F39h");


typedef union {
    struct {
        unsigned int RXB1D3 :8;
    };
    struct {
        unsigned int RXB1D30 :1;
        unsigned int RXB1D31 :1;
        unsigned int RXB1D32 :1;
        unsigned int RXB1D33 :1;
        unsigned int RXB1D34 :1;
        unsigned int RXB1D35 :1;
        unsigned int RXB1D36 :1;
        unsigned int RXB1D37 :1;
    };
} RXB1D3bits_t;
extern volatile RXB1D3bits_t RXB1D3bits __attribute__((address(0xF39)));
# 25323 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXB1D4 __attribute__((address(0xF3A)));

__asm("RXB1D4 equ 0F3Ah");


typedef union {
    struct {
        unsigned int RXB1D4 :8;
    };
    struct {
        unsigned int RXB1D40 :1;
        unsigned int RXB1D41 :1;
        unsigned int RXB1D42 :1;
        unsigned int RXB1D43 :1;
        unsigned int RXB1D44 :1;
        unsigned int RXB1D45 :1;
        unsigned int RXB1D46 :1;
        unsigned int RXB1D47 :1;
    };
} RXB1D4bits_t;
extern volatile RXB1D4bits_t RXB1D4bits __attribute__((address(0xF3A)));
# 25393 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXB1D5 __attribute__((address(0xF3B)));

__asm("RXB1D5 equ 0F3Bh");


typedef union {
    struct {
        unsigned int RXB1D5 :8;
    };
    struct {
        unsigned int RXB1D50 :1;
        unsigned int RXB1D51 :1;
        unsigned int RXB1D52 :1;
        unsigned int RXB1D53 :1;
        unsigned int RXB1D54 :1;
        unsigned int RXB1D55 :1;
        unsigned int RXB1D56 :1;
        unsigned int RXB1D57 :1;
    };
} RXB1D5bits_t;
extern volatile RXB1D5bits_t RXB1D5bits __attribute__((address(0xF3B)));
# 25463 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXB1D6 __attribute__((address(0xF3C)));

__asm("RXB1D6 equ 0F3Ch");


typedef union {
    struct {
        unsigned int RXB1D6 :8;
    };
    struct {
        unsigned int RXB1D60 :1;
        unsigned int RXB1D61 :1;
        unsigned int RXB1D62 :1;
        unsigned int RXB1D63 :1;
        unsigned int RXB1D64 :1;
        unsigned int RXB1D65 :1;
        unsigned int RXB1D66 :1;
        unsigned int RXB1D67 :1;
    };
} RXB1D6bits_t;
extern volatile RXB1D6bits_t RXB1D6bits __attribute__((address(0xF3C)));
# 25533 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXB1D7 __attribute__((address(0xF3D)));

__asm("RXB1D7 equ 0F3Dh");


typedef union {
    struct {
        unsigned int RXB1D7 :8;
    };
    struct {
        unsigned int RXB1D70 :1;
        unsigned int RXB1D71 :1;
        unsigned int RXB1D72 :1;
        unsigned int RXB1D73 :1;
        unsigned int RXB1D74 :1;
        unsigned int RXB1D75 :1;
        unsigned int RXB1D76 :1;
        unsigned int RXB1D77 :1;
    };
} RXB1D7bits_t;
extern volatile RXB1D7bits_t RXB1D7bits __attribute__((address(0xF3D)));
# 25603 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CANSTAT_RO0 __attribute__((address(0xF3E)));

__asm("CANSTAT_RO0 equ 0F3Eh");


typedef union {
    struct {
        unsigned int EICODE0 :1;
        unsigned int EICODE1_ICODE0 :1;
        unsigned int EICODE2_ICODE1 :1;
        unsigned int EICODE3_ICODE2 :1;
        unsigned int EICODE4 :1;
        unsigned int OPMODE :3;
    };
    struct {
        unsigned int :1;
        unsigned int EICODE1 :1;
        unsigned int EICODE2 :1;
        unsigned int EICODE3 :1;
        unsigned int :1;
        unsigned int OPMODE0 :1;
        unsigned int OPMODE1 :1;
        unsigned int OPMODE2 :1;
    };
    struct {
        unsigned int :1;
        unsigned int ICODE0 :1;
        unsigned int ICODE1 :1;
        unsigned int ICODE2 :1;
    };
} CANSTAT_RO0bits_t;
extern volatile CANSTAT_RO0bits_t CANSTAT_RO0bits __attribute__((address(0xF3E)));
# 25714 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CANCON_RO0 __attribute__((address(0xF3F)));

__asm("CANCON_RO0 equ 0F3Fh");


typedef union {
    struct {
        unsigned int FP0 :1;
        unsigned int WIN0_FP1 :1;
        unsigned int WIN1_FP2 :1;
        unsigned int WIN2_FP3 :1;
        unsigned int ABAT :1;
        unsigned int REQOP :3;
    };
    struct {
        unsigned int :1;
        unsigned int WIN0 :1;
        unsigned int WIN1 :1;
        unsigned int WIN2 :1;
    };
    struct {
        unsigned int :1;
        unsigned int FP1 :1;
        unsigned int FP2 :1;
        unsigned int FP3 :1;
    };
} CANCON_RO0bits_t;
extern volatile CANCON_RO0bits_t CANCON_RO0bits __attribute__((address(0xF3F)));
# 25806 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CCP5CON __attribute__((address(0xF47)));

__asm("CCP5CON equ 0F47h");


typedef union {
    struct {
        unsigned int CCP5M :4;
        unsigned int DC5B :2;
    };
    struct {
        unsigned int CCP5M0 :1;
        unsigned int CCP5M1 :1;
        unsigned int CCP5M2 :1;
        unsigned int CCP5M3 :1;
        unsigned int DC5B0 :1;
        unsigned int DC5B1 :1;
    };
    struct {
        unsigned int :4;
        unsigned int CCP5Y :1;
        unsigned int CCP5X :1;
    };
} CCP5CONbits_t;
extern volatile CCP5CONbits_t CCP5CONbits __attribute__((address(0xF47)));
# 25885 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned short CCPR5 __attribute__((address(0xF48)));

__asm("CCPR5 equ 0F48h");




extern volatile unsigned char CCPR5L __attribute__((address(0xF48)));

__asm("CCPR5L equ 0F48h");


typedef union {
    struct {
        unsigned int CCPR5L :8;
    };
} CCPR5Lbits_t;
extern volatile CCPR5Lbits_t CCPR5Lbits __attribute__((address(0xF48)));
# 25912 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CCPR5H __attribute__((address(0xF49)));

__asm("CCPR5H equ 0F49h");


typedef union {
    struct {
        unsigned int CCPR5H :8;
    };
} CCPR5Hbits_t;
extern volatile CCPR5Hbits_t CCPR5Hbits __attribute__((address(0xF49)));
# 25932 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CCP4CON __attribute__((address(0xF4A)));

__asm("CCP4CON equ 0F4Ah");


typedef union {
    struct {
        unsigned int CCP4M :4;
        unsigned int DC4B :2;
    };
    struct {
        unsigned int CCP4M0 :1;
        unsigned int CCP4M1 :1;
        unsigned int CCP4M2 :1;
        unsigned int CCP4M3 :1;
        unsigned int DC4B0 :1;
        unsigned int DC4B1 :1;
    };
    struct {
        unsigned int :4;
        unsigned int CCP4Y :1;
        unsigned int CCP4X :1;
    };
} CCP4CONbits_t;
extern volatile CCP4CONbits_t CCP4CONbits __attribute__((address(0xF4A)));
# 26011 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned short CCPR4 __attribute__((address(0xF4B)));

__asm("CCPR4 equ 0F4Bh");




extern volatile unsigned char CCPR4L __attribute__((address(0xF4B)));

__asm("CCPR4L equ 0F4Bh");


typedef union {
    struct {
        unsigned int CCPR4L :8;
    };
} CCPR4Lbits_t;
extern volatile CCPR4Lbits_t CCPR4Lbits __attribute__((address(0xF4B)));
# 26038 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CCPR4H __attribute__((address(0xF4C)));

__asm("CCPR4H equ 0F4Ch");


typedef union {
    struct {
        unsigned int CCPR4H :8;
    };
} CCPR4Hbits_t;
extern volatile CCPR4Hbits_t CCPR4Hbits __attribute__((address(0xF4C)));
# 26058 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CCP3CON __attribute__((address(0xF4D)));

__asm("CCP3CON equ 0F4Dh");


typedef union {
    struct {
        unsigned int CCP3M :4;
        unsigned int DC3B :2;
    };
    struct {
        unsigned int CCP3M0 :1;
        unsigned int CCP3M1 :1;
        unsigned int CCP3M2 :1;
        unsigned int CCP3M3 :1;
        unsigned int DC3B0 :1;
        unsigned int DC3B1 :1;
    };
    struct {
        unsigned int :4;
        unsigned int CCP3Y :1;
        unsigned int CCP3X :1;
    };
} CCP3CONbits_t;
extern volatile CCP3CONbits_t CCP3CONbits __attribute__((address(0xF4D)));
# 26137 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned short CCPR3 __attribute__((address(0xF4E)));

__asm("CCPR3 equ 0F4Eh");




extern volatile unsigned char CCPR3L __attribute__((address(0xF4E)));

__asm("CCPR3L equ 0F4Eh");


typedef union {
    struct {
        unsigned int CCPR3L :8;
    };
} CCPR3Lbits_t;
extern volatile CCPR3Lbits_t CCPR3Lbits __attribute__((address(0xF4E)));
# 26164 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CCPR3H __attribute__((address(0xF4F)));

__asm("CCPR3H equ 0F4Fh");


typedef union {
    struct {
        unsigned int CCPR3H :8;
    };
} CCPR3Hbits_t;
extern volatile CCPR3Hbits_t CCPR3Hbits __attribute__((address(0xF4F)));
# 26184 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CCP2CON __attribute__((address(0xF50)));

__asm("CCP2CON equ 0F50h");


extern volatile unsigned char ECCP2CON __attribute__((address(0xF50)));

__asm("ECCP2CON equ 0F50h");


typedef union {
    struct {
        unsigned int CCP2M :4;
        unsigned int DC2B :2;
    };
    struct {
        unsigned int CCP2M0 :1;
        unsigned int CCP2M1 :1;
        unsigned int CCP2M2 :1;
        unsigned int CCP2M3 :1;
        unsigned int DC2B0 :1;
        unsigned int DC2B1 :1;
    };
    struct {
        unsigned int :4;
        unsigned int CCP2Y :1;
        unsigned int CCP2X :1;
    };
} CCP2CONbits_t;
extern volatile CCP2CONbits_t CCP2CONbits __attribute__((address(0xF50)));
# 26266 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
typedef union {
    struct {
        unsigned int CCP2M :4;
        unsigned int DC2B :2;
    };
    struct {
        unsigned int CCP2M0 :1;
        unsigned int CCP2M1 :1;
        unsigned int CCP2M2 :1;
        unsigned int CCP2M3 :1;
        unsigned int DC2B0 :1;
        unsigned int DC2B1 :1;
    };
    struct {
        unsigned int :4;
        unsigned int CCP2Y :1;
        unsigned int CCP2X :1;
    };
} ECCP2CONbits_t;
extern volatile ECCP2CONbits_t ECCP2CONbits __attribute__((address(0xF50)));
# 26340 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned short CCPR2 __attribute__((address(0xF51)));

__asm("CCPR2 equ 0F51h");




extern volatile unsigned char CCPR2L __attribute__((address(0xF51)));

__asm("CCPR2L equ 0F51h");


typedef union {
    struct {
        unsigned int CCPR2L :8;
    };
} CCPR2Lbits_t;
extern volatile CCPR2Lbits_t CCPR2Lbits __attribute__((address(0xF51)));
# 26367 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CCPR2H __attribute__((address(0xF52)));

__asm("CCPR2H equ 0F52h");


typedef union {
    struct {
        unsigned int CCPR2H :8;
    };
} CCPR2Hbits_t;
extern volatile CCPR2Hbits_t CCPR2Hbits __attribute__((address(0xF52)));
# 26387 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CTMUICON __attribute__((address(0xF53)));

__asm("CTMUICON equ 0F53h");


typedef union {
    struct {
        unsigned int IRNG :2;
        unsigned int ITRIM :6;
    };
    struct {
        unsigned int IRNG0 :1;
        unsigned int IRNG1 :1;
        unsigned int ITRIM0 :1;
        unsigned int ITRIM1 :1;
        unsigned int ITRIM2 :1;
        unsigned int ITRIM3 :1;
        unsigned int ITRIM4 :1;
        unsigned int ITRIM5 :1;
    };
} CTMUICONbits_t;
extern volatile CTMUICONbits_t CTMUICONbits __attribute__((address(0xF53)));
# 26463 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CTMUCONL __attribute__((address(0xF54)));

__asm("CTMUCONL equ 0F54h");


typedef union {
    struct {
        unsigned int EDG1STAT :1;
        unsigned int EDG2STAT :1;
        unsigned int EDG1SEL :2;
        unsigned int EDG1POL :1;
        unsigned int EDG2SEL :2;
        unsigned int EDG2POL :1;
    };
    struct {
        unsigned int :2;
        unsigned int EDG1SEL0 :1;
        unsigned int EDG1SEL1 :1;
        unsigned int :1;
        unsigned int EDG2SEL0 :1;
        unsigned int EDG2SEL1 :1;
    };
} CTMUCONLbits_t;
extern volatile CTMUCONLbits_t CTMUCONLbits __attribute__((address(0xF54)));
# 26541 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CTMUCONH __attribute__((address(0xF55)));

__asm("CTMUCONH equ 0F55h");


typedef union {
    struct {
        unsigned int CTTRIG :1;
        unsigned int IDISSEN :1;
        unsigned int EDGSEQEN :1;
        unsigned int EDGEN :1;
        unsigned int TGEN :1;
        unsigned int CTMUSIDL :1;
        unsigned int :1;
        unsigned int CTMUEN :1;
    };
} CTMUCONHbits_t;
extern volatile CTMUCONHbits_t CTMUCONHbits __attribute__((address(0xF55)));
# 26598 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char PADCFG1 __attribute__((address(0xF56)));

__asm("PADCFG1 equ 0F56h");


typedef union {
    struct {
        unsigned int CTMUDS :1;
    };
    struct {
        unsigned int PMPTTL :1;
    };
} PADCFG1bits_t;
extern volatile PADCFG1bits_t PADCFG1bits __attribute__((address(0xF56)));
# 26626 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char PMD2 __attribute__((address(0xF57)));

__asm("PMD2 equ 0F57h");


typedef union {
    struct {
        unsigned int CMP1MD :1;
        unsigned int CMP2MD :1;
        unsigned int ECANMD :1;
    };
} PMD2bits_t;
extern volatile PMD2bits_t PMD2bits __attribute__((address(0xF57)));
# 26658 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char PMD1 __attribute__((address(0xF58)));

__asm("PMD1 equ 0F58h");


typedef union {
    struct {
        unsigned int TMR0MD :1;
        unsigned int TMR1MD :1;
        unsigned int TMR2MD :1;
        unsigned int TMR3MD :1;
        unsigned int TMR4MD :1;
        unsigned int ADCMD :1;
        unsigned int CTMUMD :1;
        unsigned int PSPMD :1;
    };
    struct {
        unsigned int EMBMD :1;
    };
} PMD1bits_t;
extern volatile PMD1bits_t PMD1bits __attribute__((address(0xF58)));
# 26728 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char PMD0 __attribute__((address(0xF59)));

__asm("PMD0 equ 0F59h");


typedef union {
    struct {
        unsigned int SSPMD :1;
        unsigned int UART1MD :1;
        unsigned int UART2MD :1;
        unsigned int CCP1MD :1;
        unsigned int CCP2MD :1;
        unsigned int CCP3MD :1;
        unsigned int CCP4MD :1;
        unsigned int CCP5MD :1;
    };
    struct {
        unsigned int :1;
        unsigned int SPI1MD :1;
        unsigned int SPI2MD :1;
    };
} PMD0bits_t;
extern volatile PMD0bits_t PMD0bits __attribute__((address(0xF59)));
# 26805 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char IOCB __attribute__((address(0xF5A)));

__asm("IOCB equ 0F5Ah");


typedef union {
    struct {
        unsigned int :4;
        unsigned int IOCB4 :1;
        unsigned int IOCB5 :1;
        unsigned int IOCB6 :1;
        unsigned int IOCB7 :1;
    };
} IOCBbits_t;
extern volatile IOCBbits_t IOCBbits __attribute__((address(0xF5A)));
# 26844 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char WPUB __attribute__((address(0xF5B)));

__asm("WPUB equ 0F5Bh");


typedef union {
    struct {
        unsigned int WPUB0 :1;
        unsigned int WPUB1 :1;
        unsigned int WPUB2 :1;
        unsigned int WPUB3 :1;
        unsigned int WPUB4 :1;
        unsigned int WPUB5 :1;
        unsigned int WPUB6 :1;
        unsigned int WPUB7 :1;
    };
} WPUBbits_t;
extern volatile WPUBbits_t WPUBbits __attribute__((address(0xF5B)));
# 26906 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char ANCON1 __attribute__((address(0xF5C)));

__asm("ANCON1 equ 0F5Ch");


typedef union {
    struct {
        unsigned int ANSEL8 :1;
        unsigned int ANSEL9 :1;
        unsigned int ANSEL10 :1;
    };
    struct {
        unsigned int PCFG8 :1;
        unsigned int PCFG9 :1;
        unsigned int PCFG10 :1;
    };
} ANCON1bits_t;
extern volatile ANCON1bits_t ANCON1bits __attribute__((address(0xF5C)));
# 26958 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char ANCON0 __attribute__((address(0xF5D)));

__asm("ANCON0 equ 0F5Dh");


typedef union {
    struct {
        unsigned int ANSEL0 :1;
        unsigned int ANSEL1 :1;
        unsigned int ANSEL2 :1;
        unsigned int ANSEL3 :1;
        unsigned int ANSEL4 :1;
    };
    struct {
        unsigned int PCFG0 :1;
        unsigned int PCFG1 :1;
        unsigned int PCFG2 :1;
        unsigned int PCFG3 :1;
        unsigned int PCFG4 :1;
    };
} ANCON0bits_t;
extern volatile ANCON0bits_t ANCON0bits __attribute__((address(0xF5D)));
# 27034 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CM2CON __attribute__((address(0xF5E)));

__asm("CM2CON equ 0F5Eh");


extern volatile unsigned char CM2CON1 __attribute__((address(0xF5E)));

__asm("CM2CON1 equ 0F5Eh");


typedef union {
    struct {
        unsigned int CCH :2;
        unsigned int CREF :1;
        unsigned int EVPOL :2;
        unsigned int CPOL :1;
        unsigned int COE :1;
        unsigned int CON :1;
    };
    struct {
        unsigned int CCH0 :1;
        unsigned int CCH1 :1;
        unsigned int :1;
        unsigned int EVPOL0 :1;
        unsigned int EVPOL1 :1;
    };
    struct {
        unsigned int CCH02 :1;
        unsigned int CCH12 :1;
        unsigned int CREF2 :1;
        unsigned int EVPOL02 :1;
        unsigned int EVPOL12 :1;
        unsigned int CPOL2 :1;
        unsigned int COE2 :1;
        unsigned int CON2 :1;
    };
} CM2CONbits_t;
extern volatile CM2CONbits_t CM2CONbits __attribute__((address(0xF5E)));
# 27164 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
typedef union {
    struct {
        unsigned int CCH :2;
        unsigned int CREF :1;
        unsigned int EVPOL :2;
        unsigned int CPOL :1;
        unsigned int COE :1;
        unsigned int CON :1;
    };
    struct {
        unsigned int CCH0 :1;
        unsigned int CCH1 :1;
        unsigned int :1;
        unsigned int EVPOL0 :1;
        unsigned int EVPOL1 :1;
    };
    struct {
        unsigned int CCH02 :1;
        unsigned int CCH12 :1;
        unsigned int CREF2 :1;
        unsigned int EVPOL02 :1;
        unsigned int EVPOL12 :1;
        unsigned int CPOL2 :1;
        unsigned int COE2 :1;
        unsigned int CON2 :1;
    };
} CM2CON1bits_t;
extern volatile CM2CON1bits_t CM2CON1bits __attribute__((address(0xF5E)));
# 27286 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CM1CON __attribute__((address(0xF5F)));

__asm("CM1CON equ 0F5Fh");


extern volatile unsigned char CM1CON1 __attribute__((address(0xF5F)));

__asm("CM1CON1 equ 0F5Fh");


typedef union {
    struct {
        unsigned int CCH :2;
        unsigned int CREF :1;
        unsigned int EVPOL :2;
        unsigned int CPOL :1;
        unsigned int COE :1;
        unsigned int CON :1;
    };
    struct {
        unsigned int CCH0 :1;
        unsigned int CCH1 :1;
        unsigned int :1;
        unsigned int EVPOL0 :1;
        unsigned int EVPOL1 :1;
    };
    struct {
        unsigned int C1CH0 :1;
        unsigned int C1CH1 :1;
        unsigned int CREF1 :1;
        unsigned int EVPOL01 :1;
        unsigned int EVPOL11 :1;
        unsigned int CPOL1 :1;
        unsigned int COE1 :1;
        unsigned int CON1 :1;
    };
    struct {
        unsigned int CCH01 :1;
        unsigned int CCH11 :1;
    };
} CM1CONbits_t;
extern volatile CM1CONbits_t CM1CONbits __attribute__((address(0xF5F)));
# 27430 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
typedef union {
    struct {
        unsigned int CCH :2;
        unsigned int CREF :1;
        unsigned int EVPOL :2;
        unsigned int CPOL :1;
        unsigned int COE :1;
        unsigned int CON :1;
    };
    struct {
        unsigned int CCH0 :1;
        unsigned int CCH1 :1;
        unsigned int :1;
        unsigned int EVPOL0 :1;
        unsigned int EVPOL1 :1;
    };
    struct {
        unsigned int C1CH0 :1;
        unsigned int C1CH1 :1;
        unsigned int CREF1 :1;
        unsigned int EVPOL01 :1;
        unsigned int EVPOL11 :1;
        unsigned int CPOL1 :1;
        unsigned int COE1 :1;
        unsigned int CON1 :1;
    };
    struct {
        unsigned int CCH01 :1;
        unsigned int CCH11 :1;
    };
} CM1CON1bits_t;
extern volatile CM1CON1bits_t CM1CON1bits __attribute__((address(0xF5F)));
# 27566 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXB0CON __attribute__((address(0xF60)));

__asm("RXB0CON equ 0F60h");


typedef union {
    struct {
        unsigned int FILHIT0 :1;
        unsigned int JTOFF_FILHIT1 :1;
        unsigned int RB0DBEN_FILHIT2 :1;
        unsigned int RXRTRRO_FILHIT3 :1;
        unsigned int FILHIT4 :1;
        unsigned int RXM0_RTRRO :1;
        unsigned int RXM1 :1;
        unsigned int RXFUL :1;
    };
    struct {
        unsigned int :1;
        unsigned int JTOFF :1;
        unsigned int RB0DBEN :1;
        unsigned int RXRTRRO :1;
        unsigned int :1;
        unsigned int RTRRO :1;
    };
    struct {
        unsigned int :1;
        unsigned int FILHIT1 :1;
        unsigned int FILHIT2 :1;
        unsigned int FILHIT3 :1;
        unsigned int :1;
        unsigned int RXM0 :1;
    };
    struct {
        unsigned int RXB0FILHIT0 :1;
        unsigned int RXB0FILHIT1 :1;
        unsigned int RXB0FILHIT2 :1;
        unsigned int RXB0FILHIT3 :1;
        unsigned int RXB0FILHIT4 :1;
        unsigned int RXB0M0 :1;
        unsigned int RXB0M1 :1;
        unsigned int RXB0FUL :1;
    };
    struct {
        unsigned int :3;
        unsigned int RXB0RTRR0 :1;
        unsigned int :1;
        unsigned int RXB0RTRRO :1;
    };
} RXB0CONbits_t;
extern volatile RXB0CONbits_t RXB0CONbits __attribute__((address(0xF60)));
# 27750 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXB0SIDH __attribute__((address(0xF61)));

__asm("RXB0SIDH equ 0F61h");


typedef union {
    struct {
        unsigned int SID :8;
    };
    struct {
        unsigned int SID3 :1;
        unsigned int SID4 :1;
        unsigned int SID5 :1;
        unsigned int SID6 :1;
        unsigned int SID7 :1;
        unsigned int SID8 :1;
        unsigned int SID9 :1;
        unsigned int SID10 :1;
    };
    struct {
        unsigned int RXB0SID3 :1;
        unsigned int RXB0SID4 :1;
        unsigned int RXB0SID5 :1;
        unsigned int RXB0SID6 :1;
        unsigned int RXB0SID7 :1;
        unsigned int RXB0SID8 :1;
        unsigned int RXB0SID9 :1;
        unsigned int RXB0SID10 :1;
    };
} RXB0SIDHbits_t;
extern volatile RXB0SIDHbits_t RXB0SIDHbits __attribute__((address(0xF61)));
# 27870 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXB0SIDL __attribute__((address(0xF62)));

__asm("RXB0SIDL equ 0F62h");


typedef union {
    struct {
        unsigned int EID :2;
        unsigned int :1;
        unsigned int EXID :1;
        unsigned int SRR :1;
        unsigned int SID :3;
    };
    struct {
        unsigned int EID16 :1;
        unsigned int EID17 :1;
        unsigned int :3;
        unsigned int SID0 :1;
        unsigned int SID1 :1;
        unsigned int SID2 :1;
    };
    struct {
        unsigned int RXB0EID16 :1;
        unsigned int RXB0EID17 :1;
        unsigned int :1;
        unsigned int RXB0EXID :1;
        unsigned int RXB0SRR :1;
        unsigned int RXB0SID0 :1;
        unsigned int RXB0SID1 :1;
        unsigned int RXB0SID2 :1;
    };
} RXB0SIDLbits_t;
extern volatile RXB0SIDLbits_t RXB0SIDLbits __attribute__((address(0xF62)));
# 27987 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXB0EIDH __attribute__((address(0xF63)));

__asm("RXB0EIDH equ 0F63h");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID8 :1;
        unsigned int EID9 :1;
        unsigned int EID10 :1;
        unsigned int EID11 :1;
        unsigned int EID12 :1;
        unsigned int EID13 :1;
        unsigned int EID14 :1;
        unsigned int EID15 :1;
    };
    struct {
        unsigned int RXB0EID8 :1;
        unsigned int RXB0EID9 :1;
        unsigned int RXB0EID10 :1;
        unsigned int RXB0EID11 :1;
        unsigned int RXB0EID12 :1;
        unsigned int RXB0EID13 :1;
        unsigned int RXB0EID14 :1;
        unsigned int RXB0EID15 :1;
    };
} RXB0EIDHbits_t;
extern volatile RXB0EIDHbits_t RXB0EIDHbits __attribute__((address(0xF63)));
# 28107 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXB0EIDL __attribute__((address(0xF64)));

__asm("RXB0EIDL equ 0F64h");


typedef union {
    struct {
        unsigned int EID :8;
    };
    struct {
        unsigned int EID0 :1;
        unsigned int EID1 :1;
        unsigned int EID2 :1;
        unsigned int EID3 :1;
        unsigned int EID4 :1;
        unsigned int EID5 :1;
        unsigned int EID6 :1;
        unsigned int EID7 :1;
    };
    struct {
        unsigned int RXB0EID0 :1;
        unsigned int RXB0EID1 :1;
        unsigned int RXB0EID2 :1;
        unsigned int RXB0EID3 :1;
        unsigned int RXB0EID4 :1;
        unsigned int RXB0EID5 :1;
        unsigned int RXB0EID6 :1;
        unsigned int RXB0EID7 :1;
    };
} RXB0EIDLbits_t;
extern volatile RXB0EIDLbits_t RXB0EIDLbits __attribute__((address(0xF64)));
# 28227 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXB0DLC __attribute__((address(0xF65)));

__asm("RXB0DLC equ 0F65h");


typedef union {
    struct {
        unsigned int DLC :4;
        unsigned int RB :2;
        unsigned int RXRTR :1;
    };
    struct {
        unsigned int DLC0 :1;
        unsigned int DLC1 :1;
        unsigned int DLC2 :1;
        unsigned int DLC3 :1;
        unsigned int RB0 :1;
        unsigned int RB1 :1;
    };
    struct {
        unsigned int RXB0DLC0 :1;
        unsigned int RXB0DLC1 :1;
        unsigned int RXB0DLC2 :1;
        unsigned int RXB0DLC3 :1;
        unsigned int RXB0RB0 :1;
        unsigned int RXB0RB1 :1;
        unsigned int RXB0RTR :1;
    };
} RXB0DLCbits_t;
extern volatile RXB0DLCbits_t RXB0DLCbits __attribute__((address(0xF65)));
# 28341 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXB0D0 __attribute__((address(0xF66)));

__asm("RXB0D0 equ 0F66h");


typedef union {
    struct {
        unsigned int RXB0D0 :8;
    };
    struct {
        unsigned int RXB0D00 :1;
        unsigned int RXB0D01 :1;
        unsigned int RXB0D02 :1;
        unsigned int RXB0D03 :1;
        unsigned int RXB0D04 :1;
        unsigned int RXB0D05 :1;
        unsigned int RXB0D06 :1;
        unsigned int RXB0D07 :1;
    };
} RXB0D0bits_t;
extern volatile RXB0D0bits_t RXB0D0bits __attribute__((address(0xF66)));
# 28411 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXB0D1 __attribute__((address(0xF67)));

__asm("RXB0D1 equ 0F67h");


typedef union {
    struct {
        unsigned int RXB0D1 :8;
    };
    struct {
        unsigned int RXB0D10 :1;
        unsigned int RXB0D11 :1;
        unsigned int RXB0D12 :1;
        unsigned int RXB0D13 :1;
        unsigned int RXB0D14 :1;
        unsigned int RXB0D15 :1;
        unsigned int RXB0D16 :1;
        unsigned int RXB0D17 :1;
    };
} RXB0D1bits_t;
extern volatile RXB0D1bits_t RXB0D1bits __attribute__((address(0xF67)));
# 28481 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXB0D2 __attribute__((address(0xF68)));

__asm("RXB0D2 equ 0F68h");


typedef union {
    struct {
        unsigned int RXB0D2 :8;
    };
    struct {
        unsigned int RXB0D20 :1;
        unsigned int RXB0D21 :1;
        unsigned int RXB0D22 :1;
        unsigned int RXB0D23 :1;
        unsigned int RXB0D24 :1;
        unsigned int RXB0D25 :1;
        unsigned int RXB0D26 :1;
        unsigned int RXB0D27 :1;
    };
} RXB0D2bits_t;
extern volatile RXB0D2bits_t RXB0D2bits __attribute__((address(0xF68)));
# 28551 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXB0D3 __attribute__((address(0xF69)));

__asm("RXB0D3 equ 0F69h");


typedef union {
    struct {
        unsigned int RXB0D3 :8;
    };
    struct {
        unsigned int RXB0D30 :1;
        unsigned int RXB0D31 :1;
        unsigned int RXB0D32 :1;
        unsigned int RXB0D33 :1;
        unsigned int RXB0D34 :1;
        unsigned int RXB0D35 :1;
        unsigned int RXB0D36 :1;
        unsigned int RXB0D37 :1;
    };
} RXB0D3bits_t;
extern volatile RXB0D3bits_t RXB0D3bits __attribute__((address(0xF69)));
# 28621 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXB0D4 __attribute__((address(0xF6A)));

__asm("RXB0D4 equ 0F6Ah");


typedef union {
    struct {
        unsigned int RXB0D4 :8;
    };
    struct {
        unsigned int RXB0D40 :1;
        unsigned int RXB0D41 :1;
        unsigned int RXB0D42 :1;
        unsigned int RXB0D43 :1;
        unsigned int RXB0D44 :1;
        unsigned int RXB0D45 :1;
        unsigned int RXB0D46 :1;
        unsigned int RXB0D47 :1;
    };
} RXB0D4bits_t;
extern volatile RXB0D4bits_t RXB0D4bits __attribute__((address(0xF6A)));
# 28691 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXB0D5 __attribute__((address(0xF6B)));

__asm("RXB0D5 equ 0F6Bh");


typedef union {
    struct {
        unsigned int RXB0D5 :8;
    };
    struct {
        unsigned int RXB0D50 :1;
        unsigned int RXB0D51 :1;
        unsigned int RXB0D52 :1;
        unsigned int RXB0D53 :1;
        unsigned int RXB0D54 :1;
        unsigned int RXB0D55 :1;
        unsigned int RXB0D56 :1;
        unsigned int RXB0D57 :1;
    };
} RXB0D5bits_t;
extern volatile RXB0D5bits_t RXB0D5bits __attribute__((address(0xF6B)));
# 28761 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXB0D6 __attribute__((address(0xF6C)));

__asm("RXB0D6 equ 0F6Ch");


typedef union {
    struct {
        unsigned int RXB0D6 :8;
    };
    struct {
        unsigned int RXB0D60 :1;
        unsigned int RXB0D61 :1;
        unsigned int RXB0D62 :1;
        unsigned int RXB0D63 :1;
        unsigned int RXB0D64 :1;
        unsigned int RXB0D65 :1;
        unsigned int RXB0D66 :1;
        unsigned int RXB0D67 :1;
    };
} RXB0D6bits_t;
extern volatile RXB0D6bits_t RXB0D6bits __attribute__((address(0xF6C)));
# 28831 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RXB0D7 __attribute__((address(0xF6D)));

__asm("RXB0D7 equ 0F6Dh");


typedef union {
    struct {
        unsigned int RXB0D7 :8;
    };
    struct {
        unsigned int RXB0D70 :1;
        unsigned int RXB0D71 :1;
        unsigned int RXB0D72 :1;
        unsigned int RXB0D73 :1;
        unsigned int RXB0D74 :1;
        unsigned int RXB0D75 :1;
        unsigned int RXB0D76 :1;
        unsigned int RXB0D77 :1;
    };
} RXB0D7bits_t;
extern volatile RXB0D7bits_t RXB0D7bits __attribute__((address(0xF6D)));
# 28901 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CANSTAT __attribute__((address(0xF6E)));

__asm("CANSTAT equ 0F6Eh");


typedef union {
    struct {
        unsigned int EICODE0 :1;
        unsigned int EICODE1_ICODE0 :1;
        unsigned int EICODE2_ICODE1 :1;
        unsigned int EICODE3_ICODE2 :1;
        unsigned int EICODE4 :1;
        unsigned int OPMODE :3;
    };
    struct {
        unsigned int :1;
        unsigned int EICODE1 :1;
        unsigned int EICODE2 :1;
        unsigned int EICODE3 :1;
        unsigned int :1;
        unsigned int OPMODE0 :1;
        unsigned int OPMODE1 :1;
        unsigned int OPMODE2 :1;
    };
    struct {
        unsigned int :1;
        unsigned int ICODE0 :1;
        unsigned int ICODE1 :1;
        unsigned int ICODE2 :1;
    };
} CANSTATbits_t;
extern volatile CANSTATbits_t CANSTATbits __attribute__((address(0xF6E)));
# 29012 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CANCON __attribute__((address(0xF6F)));

__asm("CANCON equ 0F6Fh");


typedef union {
    struct {
        unsigned int FP0 :1;
        unsigned int WIN0_FP1 :1;
        unsigned int WIN1_FP2 :1;
        unsigned int WIN2_FP3 :1;
        unsigned int ABAT :1;
        unsigned int REQOP :3;
    };
    struct {
        unsigned int :1;
        unsigned int WIN0 :1;
        unsigned int WIN1 :1;
        unsigned int WIN2 :1;
    };
    struct {
        unsigned int :1;
        unsigned int FP1 :1;
        unsigned int FP2 :1;
        unsigned int FP3 :1;
    };
} CANCONbits_t;
extern volatile CANCONbits_t CANCONbits __attribute__((address(0xF6F)));
# 29104 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CIOCON __attribute__((address(0xF70)));

__asm("CIOCON equ 0F70h");


typedef union {
    struct {
        unsigned int CLKSEL :1;
        unsigned int :3;
        unsigned int CANCAP :1;
        unsigned int ENDRHI :1;
        unsigned int TX2EN :1;
        unsigned int TX2SRC :1;
    };
} CIOCONbits_t;
extern volatile CIOCONbits_t CIOCONbits __attribute__((address(0xF70)));
# 29149 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char COMSTAT __attribute__((address(0xF71)));

__asm("COMSTAT equ 0F71h");


typedef union {
    struct {
        unsigned int EWARN :1;
        unsigned int RXWARN :1;
        unsigned int TXWARN :1;
        unsigned int RXBP :1;
        unsigned int TXBP :1;
        unsigned int TXBO :1;
        unsigned int RXB1OVFL :1;
        unsigned int RXB0OVFL :1;
    };
    struct {
        unsigned int :7;
        unsigned int NOT_FIFOEMPTY :1;
    };
    struct {
        unsigned int :7;
        unsigned int nFIFOEMPTY :1;
    };
    struct {
        unsigned int :6;
        unsigned int RXBNOVFL :1;
        unsigned int FIFOEMPTY :1;
    };
} COMSTATbits_t;
extern volatile COMSTATbits_t COMSTATbits __attribute__((address(0xF71)));
# 29244 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char ECANCON __attribute__((address(0xF72)));

__asm("ECANCON equ 0F72h");


typedef union {
    struct {
        unsigned int EWIN :5;
        unsigned int FIFOWM :1;
        unsigned int MDSEL :2;
    };
    struct {
        unsigned int EWIN0 :1;
        unsigned int EWIN1 :1;
        unsigned int EWIN2 :1;
        unsigned int EWIN3 :1;
        unsigned int EWIN4 :1;
        unsigned int :1;
        unsigned int MDSEL0 :1;
        unsigned int MDSEL1 :1;
    };
} ECANCONbits_t;
extern volatile ECANCONbits_t ECANCONbits __attribute__((address(0xF72)));
# 29321 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char EEDATA __attribute__((address(0xF73)));

__asm("EEDATA equ 0F73h");


typedef union {
    struct {
        unsigned int EEDATA :8;
    };
} EEDATAbits_t;
extern volatile EEDATAbits_t EEDATAbits __attribute__((address(0xF73)));
# 29341 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char EEADR __attribute__((address(0xF74)));

__asm("EEADR equ 0F74h");


typedef union {
    struct {
        unsigned int EEADR :8;
    };
} EEADRbits_t;
extern volatile EEADRbits_t EEADRbits __attribute__((address(0xF74)));
# 29361 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char EEADRH __attribute__((address(0xF75)));

__asm("EEADRH equ 0F75h");


typedef union {
    struct {
        unsigned int EEADRH :8;
    };
} EEADRHbits_t;
extern volatile EEADRHbits_t EEADRHbits __attribute__((address(0xF75)));
# 29381 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char PIE5 __attribute__((address(0xF76)));

__asm("PIE5 equ 0F76h");


typedef union {
    struct {
        unsigned int RXB0IE :1;
        unsigned int RXB1IE :1;
        unsigned int TXB0IE :1;
        unsigned int TXB1IE :1;
        unsigned int TXB2IE :1;
        unsigned int ERRIE :1;
        unsigned int WAKIE :1;
        unsigned int IRXIE :1;
    };
    struct {
        unsigned int FIFOWMIE :1;
        unsigned int RXBnIE :1;
        unsigned int :2;
        unsigned int TXBnIE :1;
    };
} PIE5bits_t;
extern volatile PIE5bits_t PIE5bits __attribute__((address(0xF76)));
# 29464 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char PIR5 __attribute__((address(0xF77)));

__asm("PIR5 equ 0F77h");


typedef union {
    struct {
        unsigned int RXB0IF :1;
        unsigned int RXB1IF :1;
        unsigned int TXB0IF :1;
        unsigned int TXB1IF :1;
        unsigned int TXB2IF :1;
        unsigned int ERRIF :1;
        unsigned int WAKIF :1;
        unsigned int IRXIF :1;
    };
    struct {
        unsigned int FIFOWMIF :1;
        unsigned int RXBnIF :1;
        unsigned int :2;
        unsigned int TXBnIF :1;
    };
} PIR5bits_t;
extern volatile PIR5bits_t PIR5bits __attribute__((address(0xF77)));
# 29547 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char IPR5 __attribute__((address(0xF78)));

__asm("IPR5 equ 0F78h");


typedef union {
    struct {
        unsigned int RXB0IP :1;
        unsigned int RXB1IP :1;
        unsigned int TXB0IP :1;
        unsigned int TXB1IP :1;
        unsigned int TXB2IP :1;
        unsigned int ERRIP :1;
        unsigned int WAKIP :1;
        unsigned int IRXIP :1;
    };
    struct {
        unsigned int FIFOWMIP :1;
        unsigned int RXBnIP :1;
        unsigned int :2;
        unsigned int TXBnIP :1;
    };
    struct {
        unsigned int CCH05 :1;
        unsigned int CCH15 :1;
        unsigned int :1;
        unsigned int EVPOL05 :1;
        unsigned int EVPOL15 :1;
    };
} IPR5bits_t;
extern volatile IPR5bits_t IPR5bits __attribute__((address(0xF78)));
# 29657 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXREG2 __attribute__((address(0xF79)));

__asm("TXREG2 equ 0F79h");


typedef union {
    struct {
        unsigned int TXREG2 :8;
    };
} TXREG2bits_t;
extern volatile TXREG2bits_t TXREG2bits __attribute__((address(0xF79)));
# 29677 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RCREG2 __attribute__((address(0xF7A)));

__asm("RCREG2 equ 0F7Ah");


typedef union {
    struct {
        unsigned int RCREG2 :8;
    };
} RCREG2bits_t;
extern volatile RCREG2bits_t RCREG2bits __attribute__((address(0xF7A)));
# 29697 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char SPBRG2 __attribute__((address(0xF7B)));

__asm("SPBRG2 equ 0F7Bh");


typedef union {
    struct {
        unsigned int SPBRG2 :8;
    };
} SPBRG2bits_t;
extern volatile SPBRG2bits_t SPBRG2bits __attribute__((address(0xF7B)));
# 29717 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char SPBRGH2 __attribute__((address(0xF7C)));

__asm("SPBRGH2 equ 0F7Ch");


typedef union {
    struct {
        unsigned int SPBRGH2 :8;
    };
} SPBRGH2bits_t;
extern volatile SPBRGH2bits_t SPBRGH2bits __attribute__((address(0xF7C)));
# 29737 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char SPBRGH1 __attribute__((address(0xF7D)));

__asm("SPBRGH1 equ 0F7Dh");


typedef union {
    struct {
        unsigned int SPBRGH1 :8;
    };
} SPBRGH1bits_t;
extern volatile SPBRGH1bits_t SPBRGH1bits __attribute__((address(0xF7D)));
# 29757 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char EECON2 __attribute__((address(0xF7E)));

__asm("EECON2 equ 0F7Eh");


typedef union {
    struct {
        unsigned int EECON2 :8;
    };
} EECON2bits_t;
extern volatile EECON2bits_t EECON2bits __attribute__((address(0xF7E)));
# 29777 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char EECON1 __attribute__((address(0xF7F)));

__asm("EECON1 equ 0F7Fh");


typedef union {
    struct {
        unsigned int RD :1;
        unsigned int WR :1;
        unsigned int WREN :1;
        unsigned int WRERR :1;
        unsigned int FREE :1;
        unsigned int :1;
        unsigned int CFGS :1;
        unsigned int EEPGD :1;
    };
    struct {
        unsigned int :6;
        unsigned int EEFS :1;
    };
} EECON1bits_t;
extern volatile EECON1bits_t EECON1bits __attribute__((address(0xF7F)));
# 29843 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char PORTA __attribute__((address(0xF80)));

__asm("PORTA equ 0F80h");


typedef union {
    struct {
        unsigned int RA0 :1;
        unsigned int RA1 :1;
        unsigned int RA2 :1;
        unsigned int RA3 :1;
        unsigned int :1;
        unsigned int RA5 :1;
        unsigned int RA6 :1;
        unsigned int RA7 :1;
    };
    struct {
        unsigned int ULPWUIN :1;
        unsigned int :4;
        unsigned int LVDIN :1;
        unsigned int :1;
        unsigned int RJPU :1;
    };
} PORTAbits_t;
extern volatile PORTAbits_t PORTAbits __attribute__((address(0xF80)));
# 29922 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char PORTB __attribute__((address(0xF81)));

__asm("PORTB equ 0F81h");


typedef union {
    struct {
        unsigned int RB0 :1;
        unsigned int RB1 :1;
        unsigned int RB2 :1;
        unsigned int RB3 :1;
        unsigned int RB4 :1;
        unsigned int RB5 :1;
        unsigned int RB6 :1;
        unsigned int RB7 :1;
    };
    struct {
        unsigned int :3;
        unsigned int CCP2_PA2 :1;
    };
} PORTBbits_t;
extern volatile PORTBbits_t PORTBbits __attribute__((address(0xF81)));
# 29993 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char PORTC __attribute__((address(0xF82)));

__asm("PORTC equ 0F82h");


typedef union {
    struct {
        unsigned int RC0 :1;
        unsigned int RC1 :1;
        unsigned int RC2 :1;
        unsigned int RC3 :1;
        unsigned int RC4 :1;
        unsigned int RC5 :1;
        unsigned int RC6 :1;
        unsigned int RC7 :1;
    };
    struct {
        unsigned int :1;
        unsigned int CCP2 :1;
        unsigned int PA1 :1;
    };
    struct {
        unsigned int :1;
        unsigned int PA2 :1;
    };
} PORTCbits_t;
extern volatile PORTCbits_t PORTCbits __attribute__((address(0xF82)));
# 30079 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char PORTE __attribute__((address(0xF84)));

__asm("PORTE equ 0F84h");


typedef union {
    struct {
        unsigned int :3;
        unsigned int RE3 :1;
    };
    struct {
        unsigned int :3;
        unsigned int CCP9E :1;
    };
    struct {
        unsigned int :3;
        unsigned int PC3E :1;
    };
} PORTEbits_t;
extern volatile PORTEbits_t PORTEbits __attribute__((address(0xF84)));
# 30118 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TMR4 __attribute__((address(0xF87)));

__asm("TMR4 equ 0F87h");


typedef union {
    struct {
        unsigned int TMR4 :8;
    };
} TMR4bits_t;
extern volatile TMR4bits_t TMR4bits __attribute__((address(0xF87)));
# 30138 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char T4CON __attribute__((address(0xF88)));

__asm("T4CON equ 0F88h");


typedef union {
    struct {
        unsigned int T4CKPS :2;
        unsigned int TMR4ON :1;
        unsigned int T4OUTPS :4;
    };
    struct {
        unsigned int T4CKPS0 :1;
        unsigned int T4CKPS1 :1;
        unsigned int :1;
        unsigned int T4OUTPS0 :1;
        unsigned int T4OUTPS1 :1;
        unsigned int T4OUTPS2 :1;
        unsigned int T4OUTPS3 :1;
    };
} T4CONbits_t;
extern volatile T4CONbits_t T4CONbits __attribute__((address(0xF88)));
# 30209 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char LATA __attribute__((address(0xF89)));

__asm("LATA equ 0F89h");


typedef union {
    struct {
        unsigned int LATA0 :1;
        unsigned int LATA1 :1;
        unsigned int LATA2 :1;
        unsigned int LATA3 :1;
        unsigned int :1;
        unsigned int LATA5 :1;
        unsigned int LATA6 :1;
        unsigned int LATA7 :1;
    };
    struct {
        unsigned int LA0 :1;
        unsigned int LA1 :1;
        unsigned int LA2 :1;
        unsigned int LA3 :1;
        unsigned int :1;
        unsigned int LA5 :1;
        unsigned int LA6 :1;
        unsigned int LA7 :1;
    };
} LATAbits_t;
extern volatile LATAbits_t LATAbits __attribute__((address(0xF89)));
# 30311 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char LATB __attribute__((address(0xF8A)));

__asm("LATB equ 0F8Ah");


typedef union {
    struct {
        unsigned int LATB0 :1;
        unsigned int LATB1 :1;
        unsigned int LATB2 :1;
        unsigned int LATB3 :1;
        unsigned int LATB4 :1;
        unsigned int LATB5 :1;
        unsigned int LATB6 :1;
        unsigned int LATB7 :1;
    };
    struct {
        unsigned int LB0 :1;
        unsigned int LB1 :1;
        unsigned int LB2 :1;
        unsigned int LB3 :1;
        unsigned int LB4 :1;
        unsigned int LB5 :1;
        unsigned int LB6 :1;
        unsigned int LB7 :1;
    };
} LATBbits_t;
extern volatile LATBbits_t LATBbits __attribute__((address(0xF8A)));
# 30423 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char LATC __attribute__((address(0xF8B)));

__asm("LATC equ 0F8Bh");


typedef union {
    struct {
        unsigned int LATC0 :1;
        unsigned int LATC1 :1;
        unsigned int LATC2 :1;
        unsigned int LATC3 :1;
        unsigned int LATC4 :1;
        unsigned int LATC5 :1;
        unsigned int LATC6 :1;
        unsigned int LATC7 :1;
    };
    struct {
        unsigned int LC0 :1;
        unsigned int LC1 :1;
        unsigned int LC2 :1;
        unsigned int LC3 :1;
        unsigned int LC4 :1;
        unsigned int LC5 :1;
        unsigned int LC6 :1;
        unsigned int LC7 :1;
    };
} LATCbits_t;
extern volatile LATCbits_t LATCbits __attribute__((address(0xF8B)));
# 30535 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char SLRCON __attribute__((address(0xF90)));

__asm("SLRCON equ 0F90h");


typedef union {
    struct {
        unsigned int SLRA :1;
        unsigned int SLRB :1;
        unsigned int SLRC :1;
    };
} SLRCONbits_t;
extern volatile SLRCONbits_t SLRCONbits __attribute__((address(0xF90)));
# 30567 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char ODCON __attribute__((address(0xF91)));

__asm("ODCON equ 0F91h");


typedef union {
    struct {
        unsigned int U1OD :1;
        unsigned int U2OD :1;
        unsigned int CCP1OD :1;
        unsigned int CCP2OD :1;
        unsigned int CCP3OD :1;
        unsigned int CCP4OD :1;
        unsigned int CCP5OD :1;
        unsigned int SSPOD :1;
    };
} ODCONbits_t;
extern volatile ODCONbits_t ODCONbits __attribute__((address(0xF91)));
# 30629 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TRISA __attribute__((address(0xF92)));

__asm("TRISA equ 0F92h");


typedef union {
    struct {
        unsigned int TRISA0 :1;
        unsigned int TRISA1 :1;
        unsigned int TRISA2 :1;
        unsigned int TRISA3 :1;
        unsigned int :1;
        unsigned int TRISA5 :1;
        unsigned int TRISA6 :1;
        unsigned int TRISA7 :1;
    };
} TRISAbits_t;
extern volatile TRISAbits_t TRISAbits __attribute__((address(0xF92)));
# 30686 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TRISB __attribute__((address(0xF93)));

__asm("TRISB equ 0F93h");


typedef union {
    struct {
        unsigned int TRISB0 :1;
        unsigned int TRISB1 :1;
        unsigned int TRISB2 :1;
        unsigned int TRISB3 :1;
        unsigned int TRISB4 :1;
        unsigned int TRISB5 :1;
        unsigned int TRISB6 :1;
        unsigned int TRISB7 :1;
    };
} TRISBbits_t;
extern volatile TRISBbits_t TRISBbits __attribute__((address(0xF93)));
# 30748 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TRISC __attribute__((address(0xF94)));

__asm("TRISC equ 0F94h");


typedef union {
    struct {
        unsigned int TRISC0 :1;
        unsigned int TRISC1 :1;
        unsigned int TRISC2 :1;
        unsigned int TRISC3 :1;
        unsigned int TRISC4 :1;
        unsigned int TRISC5 :1;
        unsigned int TRISC6 :1;
        unsigned int TRISC7 :1;
    };
} TRISCbits_t;
extern volatile TRISCbits_t TRISCbits __attribute__((address(0xF94)));
# 30810 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CCPTMRS __attribute__((address(0xF99)));

__asm("CCPTMRS equ 0F99h");


typedef union {
    struct {
        unsigned int C1TSEL :1;
        unsigned int C2TSEL :1;
        unsigned int C3TSEL :1;
        unsigned int C4TSEL :1;
        unsigned int C5TSEL :1;
    };
} CCPTMRSbits_t;
extern volatile CCPTMRSbits_t CCPTMRSbits __attribute__((address(0xF99)));
# 30854 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char REFOCON __attribute__((address(0xF9A)));

__asm("REFOCON equ 0F9Ah");


typedef union {
    struct {
        unsigned int RODIV :4;
        unsigned int ROSEL :1;
        unsigned int ROSSLP :1;
        unsigned int :1;
        unsigned int ROON :1;
    };
    struct {
        unsigned int RODIV0 :1;
        unsigned int RODIV1 :1;
        unsigned int RODIV2 :1;
        unsigned int RODIV3 :1;
    };
} REFOCONbits_t;
extern volatile REFOCONbits_t REFOCONbits __attribute__((address(0xF9A)));
# 30919 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char OSCTUNE __attribute__((address(0xF9B)));

__asm("OSCTUNE equ 0F9Bh");


typedef union {
    struct {
        unsigned int TUN :6;
        unsigned int PLLEN :1;
        unsigned int INTSRC :1;
    };
    struct {
        unsigned int TUN0 :1;
        unsigned int TUN1 :1;
        unsigned int TUN2 :1;
        unsigned int TUN3 :1;
        unsigned int TUN4 :1;
        unsigned int TUN5 :1;
    };
} OSCTUNEbits_t;
extern volatile OSCTUNEbits_t OSCTUNEbits __attribute__((address(0xF9B)));
# 30989 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char PSTR1CON __attribute__((address(0xF9C)));

__asm("PSTR1CON equ 0F9Ch");


typedef union {
    struct {
        unsigned int STRA :1;
        unsigned int STRB :1;
        unsigned int STRC :1;
        unsigned int STRD :1;
        unsigned int STRSYNC :1;
        unsigned int :1;
        unsigned int CMPL :2;
    };
    struct {
        unsigned int :6;
        unsigned int CMPL0 :1;
        unsigned int CMPL1 :1;
    };
} PSTR1CONbits_t;
extern volatile PSTR1CONbits_t PSTR1CONbits __attribute__((address(0xF9C)));
# 31055 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char PIE1 __attribute__((address(0xF9D)));

__asm("PIE1 equ 0F9Dh");


typedef union {
    struct {
        unsigned int TMR1IE :1;
        unsigned int TMR2IE :1;
        unsigned int TMR1GIE :1;
        unsigned int SSPIE :1;
        unsigned int TX1IE :1;
        unsigned int RC1IE :1;
        unsigned int ADIE :1;
    };
    struct {
        unsigned int :4;
        unsigned int TXIE :1;
        unsigned int RCIE :1;
    };
} PIE1bits_t;
extern volatile PIE1bits_t PIE1bits __attribute__((address(0xF9D)));
# 31126 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char PIR1 __attribute__((address(0xF9E)));

__asm("PIR1 equ 0F9Eh");


typedef union {
    struct {
        unsigned int TMR1IF :1;
        unsigned int TMR2IF :1;
        unsigned int TMR1GIF :1;
        unsigned int SSPIF :1;
        unsigned int TX1IF :1;
        unsigned int RC1IF :1;
        unsigned int ADIF :1;
    };
    struct {
        unsigned int :4;
        unsigned int TXIF :1;
        unsigned int RCIF :1;
    };
} PIR1bits_t;
extern volatile PIR1bits_t PIR1bits __attribute__((address(0xF9E)));
# 31197 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char IPR1 __attribute__((address(0xF9F)));

__asm("IPR1 equ 0F9Fh");


typedef union {
    struct {
        unsigned int TMR1IP :1;
        unsigned int TMR2IP :1;
        unsigned int TMR1GIP :1;
        unsigned int SSPIP :1;
        unsigned int TX1IP :1;
        unsigned int RC1IP :1;
        unsigned int ADIP :1;
    };
    struct {
        unsigned int :4;
        unsigned int TXIP :1;
        unsigned int RCIP :1;
    };
} IPR1bits_t;
extern volatile IPR1bits_t IPR1bits __attribute__((address(0xF9F)));
# 31268 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char PIE2 __attribute__((address(0xFA0)));

__asm("PIE2 equ 0FA0h");


typedef union {
    struct {
        unsigned int TMR3GIE :1;
        unsigned int TMR3IE :1;
        unsigned int HLVDIE :1;
        unsigned int BCLIE :1;
        unsigned int :3;
        unsigned int OSCFIE :1;
    };
    struct {
        unsigned int :2;
        unsigned int LVDIE :1;
    };
} PIE2bits_t;
extern volatile PIE2bits_t PIE2bits __attribute__((address(0xFA0)));
# 31322 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char PIR2 __attribute__((address(0xFA1)));

__asm("PIR2 equ 0FA1h");


typedef union {
    struct {
        unsigned int TMR3GIF :1;
        unsigned int TMR3IF :1;
        unsigned int HLVDIF :1;
        unsigned int BCLIF :1;
        unsigned int :3;
        unsigned int OSCFIF :1;
    };
    struct {
        unsigned int :2;
        unsigned int LVDIF :1;
    };
} PIR2bits_t;
extern volatile PIR2bits_t PIR2bits __attribute__((address(0xFA1)));
# 31376 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char IPR2 __attribute__((address(0xFA2)));

__asm("IPR2 equ 0FA2h");


typedef union {
    struct {
        unsigned int TMR3GIP :1;
        unsigned int TMR3IP :1;
        unsigned int HLVDIP :1;
        unsigned int BCLIP :1;
        unsigned int :3;
        unsigned int OSCFIP :1;
    };
    struct {
        unsigned int :2;
        unsigned int LVDIP :1;
    };
} IPR2bits_t;
extern volatile IPR2bits_t IPR2bits __attribute__((address(0xFA2)));
# 31430 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char PIE3 __attribute__((address(0xFA3)));

__asm("PIE3 equ 0FA3h");


typedef union {
    struct {
        unsigned int :1;
        unsigned int CCP1IE :1;
        unsigned int CCP2IE :1;
        unsigned int CTMUIE :1;
        unsigned int TX2IE :1;
        unsigned int RC2IE :1;
    };
    struct {
        unsigned int :1;
        unsigned int RXB1IE :1;
        unsigned int TXB0IE :1;
        unsigned int TXB1IE :1;
        unsigned int TXB2IE :1;
    };
    struct {
        unsigned int :1;
        unsigned int RXBNIE :1;
        unsigned int :2;
        unsigned int TXBNIE :1;
    };
} PIE3bits_t;
extern volatile PIE3bits_t PIE3bits __attribute__((address(0xFA3)));
# 31518 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char PIR3 __attribute__((address(0xFA4)));

__asm("PIR3 equ 0FA4h");


typedef union {
    struct {
        unsigned int :1;
        unsigned int CCP1IF :1;
        unsigned int CCP2IF :1;
        unsigned int CTMUIF :1;
        unsigned int TX2IF :1;
        unsigned int RC2IF :1;
    };
    struct {
        unsigned int :1;
        unsigned int RXBNIF :1;
        unsigned int :2;
        unsigned int TXBNIF :1;
    };
} PIR3bits_t;
extern volatile PIR3bits_t PIR3bits __attribute__((address(0xFA4)));
# 31579 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char IPR3 __attribute__((address(0xFA5)));

__asm("IPR3 equ 0FA5h");


typedef union {
    struct {
        unsigned int :1;
        unsigned int CCP1IP :1;
        unsigned int CCP2IP :1;
        unsigned int CTMUIP :1;
        unsigned int TX2IP :1;
        unsigned int RC2IP :1;
    };
    struct {
        unsigned int :1;
        unsigned int RXBNIP :1;
        unsigned int :2;
        unsigned int TXBNIP :1;
    };
} IPR3bits_t;
extern volatile IPR3bits_t IPR3bits __attribute__((address(0xFA5)));
# 31640 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RCSTA2 __attribute__((address(0xFA6)));

__asm("RCSTA2 equ 0FA6h");


typedef union {
    struct {
        unsigned int RX9D :1;
        unsigned int OERR :1;
        unsigned int FERR :1;
        unsigned int ADDEN :1;
        unsigned int CREN :1;
        unsigned int SREN :1;
        unsigned int RX9 :1;
        unsigned int SPEN :1;
    };
    struct {
        unsigned int RX9D2 :1;
        unsigned int OERR2 :1;
        unsigned int FERR2 :1;
        unsigned int ADDEN2 :1;
        unsigned int CREN2 :1;
        unsigned int SREN2 :1;
        unsigned int RX92 :1;
        unsigned int SPEN2 :1;
    };
    struct {
        unsigned int RCD82 :1;
        unsigned int :5;
        unsigned int RC8_92 :1;
    };
    struct {
        unsigned int :6;
        unsigned int RC92 :1;
    };
} RCSTA2bits_t;
extern volatile RCSTA2bits_t RCSTA2bits __attribute__((address(0xFA6)));
# 31776 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char BAUDCON1 __attribute__((address(0xFA7)));

__asm("BAUDCON1 equ 0FA7h");


typedef union {
    struct {
        unsigned int ABDEN :1;
        unsigned int WUE :1;
        unsigned int :1;
        unsigned int BRG16 :1;
        unsigned int TXCKP :1;
        unsigned int RXDTP :1;
        unsigned int RCIDL :1;
        unsigned int ABDOVF :1;
    };
    struct {
        unsigned int :6;
        unsigned int RCMT :1;
    };
    struct {
        unsigned int ABDEN1 :1;
        unsigned int WUE1 :1;
        unsigned int :1;
        unsigned int BRG161 :1;
        unsigned int CKTXP :1;
        unsigned int DTRXP :1;
        unsigned int RCIDL1 :1;
        unsigned int ABDOVF1 :1;
    };
    struct {
        unsigned int :4;
        unsigned int SCKP :1;
        unsigned int DTRXP1 :1;
        unsigned int RCMT1 :1;
    };
    struct {
        unsigned int :4;
        unsigned int SCKP1 :1;
        unsigned int RXDTP1 :1;
    };
    struct {
        unsigned int :4;
        unsigned int TXCKP1 :1;
    };
} BAUDCON1bits_t;
extern volatile BAUDCON1bits_t BAUDCON1bits __attribute__((address(0xFA7)));
# 31932 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char HLVDCON __attribute__((address(0xFA8)));

__asm("HLVDCON equ 0FA8h");


typedef union {
    struct {
        unsigned int HLVDL :4;
        unsigned int HLVDEN :1;
        unsigned int IRVST :1;
        unsigned int BGVST :1;
        unsigned int VDIRMAG :1;
    };
    struct {
        unsigned int HLVDL0 :1;
        unsigned int HLVDL1 :1;
        unsigned int HLVDL2 :1;
        unsigned int HLVDL3 :1;
    };
} HLVDCONbits_t;
extern volatile HLVDCONbits_t HLVDCONbits __attribute__((address(0xFA8)));
# 32002 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char PR4 __attribute__((address(0xFA9)));

__asm("PR4 equ 0FA9h");


typedef union {
    struct {
        unsigned int PR4 :8;
    };
} PR4bits_t;
extern volatile PR4bits_t PR4bits __attribute__((address(0xFA9)));
# 32022 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char T1GCON __attribute__((address(0xFAA)));

__asm("T1GCON equ 0FAAh");


typedef union {
    struct {
        unsigned int :3;
        unsigned int T1GGO_NOT_T1DONE :1;
    };
    struct {
        unsigned int T1GSS :2;
        unsigned int T1GVAL :1;
        unsigned int T1GGO_nT1DONE :1;
        unsigned int T1GSPM :1;
        unsigned int T1GTM :1;
        unsigned int T1GPOL :1;
        unsigned int TMR1GE :1;
    };
    struct {
        unsigned int T1GSS0 :1;
        unsigned int T1GSS1 :1;
        unsigned int :1;
        unsigned int T1GGO :1;
    };
    struct {
        unsigned int :3;
        unsigned int NOT_T1DONE :1;
    };
    struct {
        unsigned int :3;
        unsigned int nT1DONE :1;
    };
} T1GCONbits_t;
extern volatile T1GCONbits_t T1GCONbits __attribute__((address(0xFAA)));
# 32126 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RCSTA1 __attribute__((address(0xFAB)));

__asm("RCSTA1 equ 0FABh");


extern volatile unsigned char RCSTA __attribute__((address(0xFAB)));

__asm("RCSTA equ 0FABh");


typedef union {
    struct {
        unsigned int RX9D :1;
        unsigned int OERR :1;
        unsigned int FERR :1;
        unsigned int ADDEN :1;
        unsigned int CREN :1;
        unsigned int SREN :1;
        unsigned int RX9 :1;
        unsigned int SPEN :1;
    };
    struct {
        unsigned int RCD8 :1;
        unsigned int :2;
        unsigned int ADEN :1;
        unsigned int :2;
        unsigned int RC9 :1;
    };
    struct {
        unsigned int :6;
        unsigned int NOT_RC8 :1;
    };
    struct {
        unsigned int :6;
        unsigned int nRC8 :1;
    };
    struct {
        unsigned int :6;
        unsigned int RC8_9 :1;
    };
    struct {
        unsigned int RX9D1 :1;
        unsigned int OERR1 :1;
        unsigned int FERR1 :1;
        unsigned int ADDEN1 :1;
        unsigned int CREN1 :1;
        unsigned int SREN1 :1;
        unsigned int RX91 :1;
        unsigned int SPEN1 :1;
    };
    struct {
        unsigned int :5;
        unsigned int SRENA :1;
    };
} RCSTA1bits_t;
extern volatile RCSTA1bits_t RCSTA1bits __attribute__((address(0xFAB)));
# 32299 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
typedef union {
    struct {
        unsigned int RX9D :1;
        unsigned int OERR :1;
        unsigned int FERR :1;
        unsigned int ADDEN :1;
        unsigned int CREN :1;
        unsigned int SREN :1;
        unsigned int RX9 :1;
        unsigned int SPEN :1;
    };
    struct {
        unsigned int RCD8 :1;
        unsigned int :2;
        unsigned int ADEN :1;
        unsigned int :2;
        unsigned int RC9 :1;
    };
    struct {
        unsigned int :6;
        unsigned int NOT_RC8 :1;
    };
    struct {
        unsigned int :6;
        unsigned int nRC8 :1;
    };
    struct {
        unsigned int :6;
        unsigned int RC8_9 :1;
    };
    struct {
        unsigned int RX9D1 :1;
        unsigned int OERR1 :1;
        unsigned int FERR1 :1;
        unsigned int ADDEN1 :1;
        unsigned int CREN1 :1;
        unsigned int SREN1 :1;
        unsigned int RX91 :1;
        unsigned int SPEN1 :1;
    };
    struct {
        unsigned int :5;
        unsigned int SRENA :1;
    };
} RCSTAbits_t;
extern volatile RCSTAbits_t RCSTAbits __attribute__((address(0xFAB)));
# 32464 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXSTA1 __attribute__((address(0xFAC)));

__asm("TXSTA1 equ 0FACh");


extern volatile unsigned char TXSTA __attribute__((address(0xFAC)));

__asm("TXSTA equ 0FACh");


typedef union {
    struct {
        unsigned int TX9D :1;
        unsigned int TRMT :1;
        unsigned int BRGH :1;
        unsigned int SENDB :1;
        unsigned int SYNC :1;
        unsigned int TXEN :1;
        unsigned int TX9 :1;
        unsigned int CSRC :1;
    };
    struct {
        unsigned int TXD8 :1;
        unsigned int :5;
        unsigned int TX8_9 :1;
    };
    struct {
        unsigned int :6;
        unsigned int NOT_TX8 :1;
    };
    struct {
        unsigned int :6;
        unsigned int nTX8 :1;
    };
    struct {
        unsigned int TX9D1 :1;
        unsigned int TRMT1 :1;
        unsigned int BRGH1 :1;
        unsigned int SENDB1 :1;
        unsigned int SYNC1 :1;
        unsigned int TXEN1 :1;
        unsigned int TX91 :1;
        unsigned int CSRC1 :1;
    };
} TXSTA1bits_t;
extern volatile TXSTA1bits_t TXSTA1bits __attribute__((address(0xFAC)));
# 32612 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
typedef union {
    struct {
        unsigned int TX9D :1;
        unsigned int TRMT :1;
        unsigned int BRGH :1;
        unsigned int SENDB :1;
        unsigned int SYNC :1;
        unsigned int TXEN :1;
        unsigned int TX9 :1;
        unsigned int CSRC :1;
    };
    struct {
        unsigned int TXD8 :1;
        unsigned int :5;
        unsigned int TX8_9 :1;
    };
    struct {
        unsigned int :6;
        unsigned int NOT_TX8 :1;
    };
    struct {
        unsigned int :6;
        unsigned int nTX8 :1;
    };
    struct {
        unsigned int TX9D1 :1;
        unsigned int TRMT1 :1;
        unsigned int BRGH1 :1;
        unsigned int SENDB1 :1;
        unsigned int SYNC1 :1;
        unsigned int TXEN1 :1;
        unsigned int TX91 :1;
        unsigned int CSRC1 :1;
    };
} TXSTAbits_t;
extern volatile TXSTAbits_t TXSTAbits __attribute__((address(0xFAC)));
# 32752 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXREG1 __attribute__((address(0xFAD)));

__asm("TXREG1 equ 0FADh");


extern volatile unsigned char TXREG __attribute__((address(0xFAD)));

__asm("TXREG equ 0FADh");


typedef union {
    struct {
        unsigned int TXREG1 :8;
    };
} TXREG1bits_t;
extern volatile TXREG1bits_t TXREG1bits __attribute__((address(0xFAD)));







typedef union {
    struct {
        unsigned int TXREG1 :8;
    };
} TXREGbits_t;
extern volatile TXREGbits_t TXREGbits __attribute__((address(0xFAD)));
# 32790 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RCREG1 __attribute__((address(0xFAE)));

__asm("RCREG1 equ 0FAEh");


extern volatile unsigned char RCREG __attribute__((address(0xFAE)));

__asm("RCREG equ 0FAEh");


typedef union {
    struct {
        unsigned int RCREG1 :8;
    };
} RCREG1bits_t;
extern volatile RCREG1bits_t RCREG1bits __attribute__((address(0xFAE)));







typedef union {
    struct {
        unsigned int RCREG1 :8;
    };
} RCREGbits_t;
extern volatile RCREGbits_t RCREGbits __attribute__((address(0xFAE)));
# 32828 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char SPBRG1 __attribute__((address(0xFAF)));

__asm("SPBRG1 equ 0FAFh");


extern volatile unsigned char SPBRG __attribute__((address(0xFAF)));

__asm("SPBRG equ 0FAFh");


typedef union {
    struct {
        unsigned int SPBRG1 :8;
    };
} SPBRG1bits_t;
extern volatile SPBRG1bits_t SPBRG1bits __attribute__((address(0xFAF)));







typedef union {
    struct {
        unsigned int SPBRG1 :8;
    };
} SPBRGbits_t;
extern volatile SPBRGbits_t SPBRGbits __attribute__((address(0xFAF)));
# 32866 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char T3GCON __attribute__((address(0xFB0)));

__asm("T3GCON equ 0FB0h");


typedef union {
    struct {
        unsigned int :3;
        unsigned int T3GGO_NOT_T3DONE :1;
    };
    struct {
        unsigned int T3GSS :2;
        unsigned int T3GVAL :1;
        unsigned int T3GGO_nT3DONE :1;
        unsigned int T3GSPM :1;
        unsigned int T3GTM :1;
        unsigned int T3GPOL :1;
        unsigned int TMR3GE :1;
    };
    struct {
        unsigned int T3GSS0 :1;
        unsigned int T3GSS1 :1;
        unsigned int :1;
        unsigned int T3GGO :1;
    };
    struct {
        unsigned int :3;
        unsigned int NOT_T3DONE :1;
    };
    struct {
        unsigned int :3;
        unsigned int nT3DONE :1;
    };
} T3GCONbits_t;
extern volatile T3GCONbits_t T3GCONbits __attribute__((address(0xFB0)));
# 32970 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char T3CON __attribute__((address(0xFB1)));

__asm("T3CON equ 0FB1h");


typedef union {
    struct {
        unsigned int :2;
        unsigned int NOT_T3SYNC :1;
    };
    struct {
        unsigned int TMR3ON :1;
        unsigned int RD16 :1;
        unsigned int nT3SYNC :1;
        unsigned int SOSCEN :1;
        unsigned int T3CKPS :2;
        unsigned int TMR3CS :2;
    };
    struct {
        unsigned int :4;
        unsigned int T3CKPS0 :1;
        unsigned int T3CKPS1 :1;
        unsigned int TMR3CS0 :1;
        unsigned int TMR3CS1 :1;
    };
    struct {
        unsigned int :3;
        unsigned int SOSCEN3 :1;
        unsigned int :3;
        unsigned int RD163 :1;
    };
    struct {
        unsigned int :7;
        unsigned int T3RD16 :1;
    };
} T3CONbits_t;
extern volatile T3CONbits_t T3CONbits __attribute__((address(0xFB1)));
# 33081 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned short TMR3 __attribute__((address(0xFB2)));

__asm("TMR3 equ 0FB2h");




extern volatile unsigned char TMR3L __attribute__((address(0xFB2)));

__asm("TMR3L equ 0FB2h");


typedef union {
    struct {
        unsigned int TMR3L :8;
    };
} TMR3Lbits_t;
extern volatile TMR3Lbits_t TMR3Lbits __attribute__((address(0xFB2)));
# 33108 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TMR3H __attribute__((address(0xFB3)));

__asm("TMR3H equ 0FB3h");


typedef union {
    struct {
        unsigned int TMR3H :8;
    };
} TMR3Hbits_t;
extern volatile TMR3Hbits_t TMR3Hbits __attribute__((address(0xFB3)));
# 33128 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CMSTAT __attribute__((address(0xFB4)));

__asm("CMSTAT equ 0FB4h");


extern volatile unsigned char CMSTATUS __attribute__((address(0xFB4)));

__asm("CMSTATUS equ 0FB4h");


typedef union {
    struct {
        unsigned int :6;
        unsigned int CMP1OUT :1;
        unsigned int CMP2OUT :1;
    };
    struct {
        unsigned int :6;
        unsigned int C2OUT :1;
        unsigned int C3OUT :1;
    };
} CMSTATbits_t;
extern volatile CMSTATbits_t CMSTATbits __attribute__((address(0xFB4)));
# 33173 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
typedef union {
    struct {
        unsigned int :6;
        unsigned int CMP1OUT :1;
        unsigned int CMP2OUT :1;
    };
    struct {
        unsigned int :6;
        unsigned int C2OUT :1;
        unsigned int C3OUT :1;
    };
} CMSTATUSbits_t;
extern volatile CMSTATUSbits_t CMSTATUSbits __attribute__((address(0xFB4)));
# 33210 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CVRCON __attribute__((address(0xFB5)));

__asm("CVRCON equ 0FB5h");


typedef union {
    struct {
        unsigned int CVR :5;
        unsigned int CVRSS :1;
        unsigned int CVROE :1;
        unsigned int CVREN :1;
    };
    struct {
        unsigned int CVR0 :1;
        unsigned int CVR1 :1;
        unsigned int CVR2 :1;
        unsigned int CVR3 :1;
        unsigned int CVR4 :1;
    };
    struct {
        unsigned int :5;
        unsigned int CVRR :1;
        unsigned int CVROEN :1;
    };
} CVRCONbits_t;
extern volatile CVRCONbits_t CVRCONbits __attribute__((address(0xFB5)));
# 33295 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char PIE4 __attribute__((address(0xFB6)));

__asm("PIE4 equ 0FB6h");


typedef union {
    struct {
        unsigned int CCP3IE :1;
        unsigned int CCP4IE :1;
        unsigned int CCP5IE :1;
        unsigned int :1;
        unsigned int CMP1IE :1;
        unsigned int CMP2IE :1;
        unsigned int EEIE :1;
        unsigned int TMR4IE :1;
    };
} PIE4bits_t;
extern volatile PIE4bits_t PIE4bits __attribute__((address(0xFB6)));
# 33352 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char PIR4 __attribute__((address(0xFB7)));

__asm("PIR4 equ 0FB7h");


typedef union {
    struct {
        unsigned int CCP3IF :1;
        unsigned int CCP4IF :1;
        unsigned int CCP5IF :1;
        unsigned int :1;
        unsigned int CMP1IF :1;
        unsigned int CMP2IF :1;
        unsigned int EEIF :1;
        unsigned int TMR4IF :1;
    };
} PIR4bits_t;
extern volatile PIR4bits_t PIR4bits __attribute__((address(0xFB7)));
# 33409 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char IPR4 __attribute__((address(0xFB8)));

__asm("IPR4 equ 0FB8h");


typedef union {
    struct {
        unsigned int CCP3IP :1;
        unsigned int CCP4IP :1;
        unsigned int CCP5IP :1;
        unsigned int :1;
        unsigned int CMP1IP :1;
        unsigned int CMP2IP :1;
        unsigned int EEIP :1;
        unsigned int TMR4IP :1;
    };
    struct {
        unsigned int CCIP3IP :1;
    };
} IPR4bits_t;
extern volatile IPR4bits_t IPR4bits __attribute__((address(0xFB8)));
# 33474 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char BAUDCON2 __attribute__((address(0xFB9)));

__asm("BAUDCON2 equ 0FB9h");


typedef union {
    struct {
        unsigned int ABDEN :1;
        unsigned int WUE :1;
        unsigned int :1;
        unsigned int BRG16 :1;
        unsigned int TXCKP :1;
        unsigned int RXDTP :1;
        unsigned int RCIDL :1;
        unsigned int ABDOVF :1;
    };
    struct {
        unsigned int :6;
        unsigned int RCMT :1;
    };
    struct {
        unsigned int ABDEN2 :1;
        unsigned int WUE2 :1;
        unsigned int :1;
        unsigned int BRG162 :1;
        unsigned int SCKP2 :1;
        unsigned int DTRXP2 :1;
        unsigned int RCIDL2 :1;
        unsigned int ABDOVF2 :1;
    };
    struct {
        unsigned int :4;
        unsigned int TXCKP2 :1;
        unsigned int RXDTP2 :1;
        unsigned int RCMT2 :1;
    };
} BAUDCON2bits_t;
extern volatile BAUDCON2bits_t BAUDCON2bits __attribute__((address(0xFB9)));
# 33606 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TXSTA2 __attribute__((address(0xFBA)));

__asm("TXSTA2 equ 0FBAh");


typedef union {
    struct {
        unsigned int TX9D :1;
        unsigned int TRMT :1;
        unsigned int BRGH :1;
        unsigned int SENDB :1;
        unsigned int SYNC :1;
        unsigned int TXEN :1;
        unsigned int TX9 :1;
        unsigned int CSRC :1;
    };
    struct {
        unsigned int TX9D2 :1;
        unsigned int TRMT2 :1;
        unsigned int BRGH2 :1;
        unsigned int SENDB2 :1;
        unsigned int SYNC2 :1;
        unsigned int TXEN2 :1;
        unsigned int TX92 :1;
        unsigned int CSRC2 :1;
    };
    struct {
        unsigned int TXD82 :1;
        unsigned int :5;
        unsigned int TX8_92 :1;
    };
} TXSTA2bits_t;
extern volatile TXSTA2bits_t TXSTA2bits __attribute__((address(0xFBA)));
# 33733 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CCP1CON __attribute__((address(0xFBB)));

__asm("CCP1CON equ 0FBBh");


extern volatile unsigned char ECCP1CON __attribute__((address(0xFBB)));

__asm("ECCP1CON equ 0FBBh");


typedef union {
    struct {
        unsigned int CCP1M :4;
        unsigned int DC1B :2;
        unsigned int P1M :2;
    };
    struct {
        unsigned int CCP1M0 :1;
        unsigned int CCP1M1 :1;
        unsigned int CCP1M2 :1;
        unsigned int CCP1M3 :1;
        unsigned int DC1B0 :1;
        unsigned int DC1B1 :1;
        unsigned int P1M0 :1;
        unsigned int P1M1 :1;
    };
    struct {
        unsigned int :4;
        unsigned int CCP1Y :1;
        unsigned int CCP1X :1;
    };
} CCP1CONbits_t;
extern volatile CCP1CONbits_t CCP1CONbits __attribute__((address(0xFBB)));
# 33833 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
typedef union {
    struct {
        unsigned int CCP1M :4;
        unsigned int DC1B :2;
        unsigned int P1M :2;
    };
    struct {
        unsigned int CCP1M0 :1;
        unsigned int CCP1M1 :1;
        unsigned int CCP1M2 :1;
        unsigned int CCP1M3 :1;
        unsigned int DC1B0 :1;
        unsigned int DC1B1 :1;
        unsigned int P1M0 :1;
        unsigned int P1M1 :1;
    };
    struct {
        unsigned int :4;
        unsigned int CCP1Y :1;
        unsigned int CCP1X :1;
    };
} ECCP1CONbits_t;
extern volatile ECCP1CONbits_t ECCP1CONbits __attribute__((address(0xFBB)));
# 33925 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned short CCPR1 __attribute__((address(0xFBC)));

__asm("CCPR1 equ 0FBCh");




extern volatile unsigned char CCPR1L __attribute__((address(0xFBC)));

__asm("CCPR1L equ 0FBCh");


typedef union {
    struct {
        unsigned int CCPR1L :8;
    };
} CCPR1Lbits_t;
extern volatile CCPR1Lbits_t CCPR1Lbits __attribute__((address(0xFBC)));
# 33952 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char CCPR1H __attribute__((address(0xFBD)));

__asm("CCPR1H equ 0FBDh");


typedef union {
    struct {
        unsigned int CCPR1H :8;
    };
} CCPR1Hbits_t;
extern volatile CCPR1Hbits_t CCPR1Hbits __attribute__((address(0xFBD)));
# 33972 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char ECCP1DEL __attribute__((address(0xFBE)));

__asm("ECCP1DEL equ 0FBEh");


extern volatile unsigned char PWM1CON __attribute__((address(0xFBE)));

__asm("PWM1CON equ 0FBEh");


typedef union {
    struct {
        unsigned int P1DC :7;
        unsigned int P1RSEN :1;
    };
    struct {
        unsigned int P1DC0 :1;
        unsigned int P1DC1 :1;
        unsigned int P1DC2 :1;
        unsigned int P1DC3 :1;
        unsigned int P1DC4 :1;
        unsigned int P1DC5 :1;
        unsigned int P1DC6 :1;
    };
} ECCP1DELbits_t;
extern volatile ECCP1DELbits_t ECCP1DELbits __attribute__((address(0xFBE)));
# 34045 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
typedef union {
    struct {
        unsigned int P1DC :7;
        unsigned int P1RSEN :1;
    };
    struct {
        unsigned int P1DC0 :1;
        unsigned int P1DC1 :1;
        unsigned int P1DC2 :1;
        unsigned int P1DC3 :1;
        unsigned int P1DC4 :1;
        unsigned int P1DC5 :1;
        unsigned int P1DC6 :1;
    };
} PWM1CONbits_t;
extern volatile PWM1CONbits_t PWM1CONbits __attribute__((address(0xFBE)));
# 34110 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char ECCP1AS __attribute__((address(0xFBF)));

__asm("ECCP1AS equ 0FBFh");


typedef union {
    struct {
        unsigned int PSS1BD :2;
        unsigned int PSS1AC :2;
        unsigned int ECCP1AS :3;
        unsigned int ECCP1ASE :1;
    };
    struct {
        unsigned int PSS1BD0 :1;
        unsigned int PSS1BD1 :1;
        unsigned int PSS1AC0 :1;
        unsigned int PSS1AC1 :1;
        unsigned int ECCP1AS0 :1;
        unsigned int ECCP1AS1 :1;
        unsigned int ECCP1AS2 :1;
    };
} ECCP1ASbits_t;
extern volatile ECCP1ASbits_t ECCP1ASbits __attribute__((address(0xFBF)));
# 34192 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char ADCON2 __attribute__((address(0xFC0)));

__asm("ADCON2 equ 0FC0h");


typedef union {
    struct {
        unsigned int ADCS :3;
        unsigned int ACQT :3;
        unsigned int :1;
        unsigned int ADFM :1;
    };
    struct {
        unsigned int ADCS0 :1;
        unsigned int ADCS1 :1;
        unsigned int ADCS2 :1;
        unsigned int ACQT0 :1;
        unsigned int ACQT1 :1;
        unsigned int ACQT2 :1;
    };
} ADCON2bits_t;
extern volatile ADCON2bits_t ADCON2bits __attribute__((address(0xFC0)));
# 34263 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char ADCON1 __attribute__((address(0xFC1)));

__asm("ADCON1 equ 0FC1h");


typedef union {
    struct {
        unsigned int CHSN :3;
        unsigned int VNCFG :1;
        unsigned int VCFG :2;
        unsigned int TRIGSEL :2;
    };
    struct {
        unsigned int CHSN0 :1;
        unsigned int CHSN1 :1;
        unsigned int CHSN2 :1;
        unsigned int :1;
        unsigned int VCFG0 :1;
        unsigned int VCFG1 :1;
        unsigned int TRIGSEL0 :1;
        unsigned int TRIGSEL1 :1;
    };
    struct {
        unsigned int :3;
        unsigned int CHSN3 :1;
        unsigned int VCFG01 :1;
        unsigned int VCFG11 :1;
    };
} ADCON1bits_t;
extern volatile ADCON1bits_t ADCON1bits __attribute__((address(0xFC1)));
# 34367 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char ADCON0 __attribute__((address(0xFC2)));

__asm("ADCON0 equ 0FC2h");


typedef union {
    struct {
        unsigned int :1;
        unsigned int GO_NOT_DONE :1;
    };
    struct {
        unsigned int ADON :1;
        unsigned int GO_nDONE :1;
        unsigned int CHS :5;
    };
    struct {
        unsigned int :1;
        unsigned int DONE :1;
        unsigned int CHS0 :1;
        unsigned int CHS1 :1;
        unsigned int CHS2 :1;
        unsigned int CHS3 :1;
        unsigned int CHS4 :1;
    };
    struct {
        unsigned int :1;
        unsigned int GO :1;
    };
    struct {
        unsigned int :1;
        unsigned int NOT_DONE :1;
    };
    struct {
        unsigned int :1;
        unsigned int nDONE :1;
    };
    struct {
        unsigned int :1;
        unsigned int GODONE :1;
    };
    struct {
        unsigned int :1;
        unsigned int GO_DONE :1;
    };
} ADCON0bits_t;
extern volatile ADCON0bits_t ADCON0bits __attribute__((address(0xFC2)));
# 34492 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned short ADRES __attribute__((address(0xFC3)));

__asm("ADRES equ 0FC3h");




extern volatile unsigned char ADRESL __attribute__((address(0xFC3)));

__asm("ADRESL equ 0FC3h");


typedef union {
    struct {
        unsigned int ADRESL :8;
    };
} ADRESLbits_t;
extern volatile ADRESLbits_t ADRESLbits __attribute__((address(0xFC3)));
# 34519 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char ADRESH __attribute__((address(0xFC4)));

__asm("ADRESH equ 0FC4h");


typedef union {
    struct {
        unsigned int ADRESH :8;
    };
} ADRESHbits_t;
extern volatile ADRESHbits_t ADRESHbits __attribute__((address(0xFC4)));
# 34539 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char SSPCON2 __attribute__((address(0xFC5)));

__asm("SSPCON2 equ 0FC5h");


typedef union {
    struct {
        unsigned int SEN :1;
        unsigned int RSEN :1;
        unsigned int PEN :1;
        unsigned int RCEN :1;
        unsigned int ACKEN :1;
        unsigned int ACKDT :1;
        unsigned int ACKSTAT :1;
        unsigned int GCEN :1;
    };
    struct {
        unsigned int :1;
        unsigned int ADMSK1 :1;
        unsigned int ADMSK2 :1;
        unsigned int ADMSK3 :1;
        unsigned int ADMSK4 :1;
        unsigned int ADMSK5 :1;
    };
} SSPCON2bits_t;
extern volatile SSPCON2bits_t SSPCON2bits __attribute__((address(0xFC5)));
# 34634 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char SSPCON1 __attribute__((address(0xFC6)));

__asm("SSPCON1 equ 0FC6h");


typedef union {
    struct {
        unsigned int SSPM :4;
        unsigned int CKP :1;
        unsigned int SSPEN :1;
        unsigned int SSPOV :1;
        unsigned int WCOL :1;
    };
    struct {
        unsigned int SSPM0 :1;
        unsigned int SSPM1 :1;
        unsigned int SSPM2 :1;
        unsigned int SSPM3 :1;
    };
} SSPCON1bits_t;
extern volatile SSPCON1bits_t SSPCON1bits __attribute__((address(0xFC6)));
# 34704 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char SSPSTAT __attribute__((address(0xFC7)));

__asm("SSPSTAT equ 0FC7h");


typedef union {
    struct {
        unsigned int :2;
        unsigned int R_NOT_W :1;
    };
    struct {
        unsigned int :5;
        unsigned int D_NOT_A :1;
    };
    struct {
        unsigned int BF :1;
        unsigned int UA :1;
        unsigned int R_nW :1;
        unsigned int S :1;
        unsigned int P :1;
        unsigned int D_nA :1;
        unsigned int CKE :1;
        unsigned int SMP :1;
    };
    struct {
        unsigned int :2;
        unsigned int R_W :1;
        unsigned int :2;
        unsigned int D_A :1;
    };
    struct {
        unsigned int :2;
        unsigned int nW :1;
        unsigned int :2;
        unsigned int nA :1;
    };
    struct {
        unsigned int :2;
        unsigned int NOT_WRITE :1;
    };
    struct {
        unsigned int :5;
        unsigned int NOT_ADDRESS :1;
    };
    struct {
        unsigned int :2;
        unsigned int nWRITE :1;
        unsigned int :2;
        unsigned int nADDRESS :1;
    };
    struct {
        unsigned int :2;
        unsigned int READ_WRITE :1;
        unsigned int :2;
        unsigned int DATA_ADDRESS :1;
    };
    struct {
        unsigned int :2;
        unsigned int I2C_READ :1;
        unsigned int I2C_START :1;
        unsigned int I2C_STOP :1;
        unsigned int I2C_DAT :1;
    };
    struct {
        unsigned int :2;
        unsigned int RW :1;
        unsigned int START :1;
        unsigned int STOP :1;
        unsigned int DA :1;
    };
    struct {
        unsigned int :2;
        unsigned int NOT_W :1;
        unsigned int :2;
        unsigned int NOT_A :1;
    };
} SSPSTATbits_t;
extern volatile SSPSTATbits_t SSPSTATbits __attribute__((address(0xFC7)));
# 34936 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char SSPADD __attribute__((address(0xFC8)));

__asm("SSPADD equ 0FC8h");


typedef union {
    struct {
        unsigned int SSPADD :8;
    };
    struct {
        unsigned int MSK0 :1;
        unsigned int MSK1 :1;
        unsigned int MSK2 :1;
        unsigned int MSK3 :1;
        unsigned int MSK4 :1;
        unsigned int MSK5 :1;
        unsigned int MSK6 :1;
        unsigned int MSK7 :1;
    };
} SSPADDbits_t;
extern volatile SSPADDbits_t SSPADDbits __attribute__((address(0xFC8)));
# 35006 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char SSPBUF __attribute__((address(0xFC9)));

__asm("SSPBUF equ 0FC9h");


typedef union {
    struct {
        unsigned int SSPBUF :8;
    };
} SSPBUFbits_t;
extern volatile SSPBUFbits_t SSPBUFbits __attribute__((address(0xFC9)));
# 35026 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char T2CON __attribute__((address(0xFCA)));

__asm("T2CON equ 0FCAh");


typedef union {
    struct {
        unsigned int T2CKPS :2;
        unsigned int TMR2ON :1;
        unsigned int T2OUTPS :4;
    };
    struct {
        unsigned int T2CKPS0 :1;
        unsigned int T2CKPS1 :1;
        unsigned int :1;
        unsigned int T2OUTPS0 :1;
        unsigned int T2OUTPS1 :1;
        unsigned int T2OUTPS2 :1;
        unsigned int T2OUTPS3 :1;
    };
} T2CONbits_t;
extern volatile T2CONbits_t T2CONbits __attribute__((address(0xFCA)));
# 35097 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char PR2 __attribute__((address(0xFCB)));

__asm("PR2 equ 0FCBh");


extern volatile unsigned char MEMCON __attribute__((address(0xFCB)));

__asm("MEMCON equ 0FCBh");


typedef union {
    struct {
        unsigned int PR2 :8;
    };
    struct {
        unsigned int :7;
        unsigned int EBDIS :1;
    };
    struct {
        unsigned int :4;
        unsigned int WAIT0 :1;
    };
    struct {
        unsigned int :5;
        unsigned int WAIT1 :1;
    };
    struct {
        unsigned int WM0 :1;
    };
    struct {
        unsigned int :1;
        unsigned int WM1 :1;
    };
} PR2bits_t;
extern volatile PR2bits_t PR2bits __attribute__((address(0xFCB)));
# 35164 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
typedef union {
    struct {
        unsigned int PR2 :8;
    };
    struct {
        unsigned int :7;
        unsigned int EBDIS :1;
    };
    struct {
        unsigned int :4;
        unsigned int WAIT0 :1;
    };
    struct {
        unsigned int :5;
        unsigned int WAIT1 :1;
    };
    struct {
        unsigned int WM0 :1;
    };
    struct {
        unsigned int :1;
        unsigned int WM1 :1;
    };
} MEMCONbits_t;
extern volatile MEMCONbits_t MEMCONbits __attribute__((address(0xFCB)));
# 35223 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TMR2 __attribute__((address(0xFCC)));

__asm("TMR2 equ 0FCCh");


typedef union {
    struct {
        unsigned int TMR2 :8;
    };
} TMR2bits_t;
extern volatile TMR2bits_t TMR2bits __attribute__((address(0xFCC)));
# 35243 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char T1CON __attribute__((address(0xFCD)));

__asm("T1CON equ 0FCDh");


typedef union {
    struct {
        unsigned int :2;
        unsigned int NOT_T1SYNC :1;
    };
    struct {
        unsigned int TMR1ON :1;
        unsigned int RD16 :1;
        unsigned int nT1SYNC :1;
        unsigned int SOSCEN :1;
        unsigned int T1CKPS :2;
        unsigned int TMR1CS :2;
    };
    struct {
        unsigned int :4;
        unsigned int T1CKPS0 :1;
        unsigned int T1CKPS1 :1;
        unsigned int TMR1CS0 :1;
        unsigned int TMR1CS1 :1;
    };
    struct {
        unsigned int :3;
        unsigned int T1OSCEN :1;
        unsigned int :3;
        unsigned int T1RD16 :1;
    };
} T1CONbits_t;
extern volatile T1CONbits_t T1CONbits __attribute__((address(0xFCD)));
# 35345 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned short TMR1 __attribute__((address(0xFCE)));

__asm("TMR1 equ 0FCEh");




extern volatile unsigned char TMR1L __attribute__((address(0xFCE)));

__asm("TMR1L equ 0FCEh");


typedef union {
    struct {
        unsigned int TMR1L :8;
    };
} TMR1Lbits_t;
extern volatile TMR1Lbits_t TMR1Lbits __attribute__((address(0xFCE)));
# 35372 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TMR1H __attribute__((address(0xFCF)));

__asm("TMR1H equ 0FCFh");


typedef union {
    struct {
        unsigned int TMR1H :8;
    };
} TMR1Hbits_t;
extern volatile TMR1Hbits_t TMR1Hbits __attribute__((address(0xFCF)));
# 35392 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char RCON __attribute__((address(0xFD0)));

__asm("RCON equ 0FD0h");


typedef union {
    struct {
        unsigned int NOT_BOR :1;
    };
    struct {
        unsigned int :1;
        unsigned int NOT_POR :1;
    };
    struct {
        unsigned int :2;
        unsigned int NOT_PD :1;
    };
    struct {
        unsigned int :3;
        unsigned int NOT_TO :1;
    };
    struct {
        unsigned int :4;
        unsigned int NOT_RI :1;
    };
    struct {
        unsigned int :5;
        unsigned int NOT_CM :1;
    };
    struct {
        unsigned int nBOR :1;
        unsigned int nPOR :1;
        unsigned int nPD :1;
        unsigned int nTO :1;
        unsigned int nRI :1;
        unsigned int nCM :1;
        unsigned int SBOREN :1;
        unsigned int IPEN :1;
    };
    struct {
        unsigned int BOR :1;
        unsigned int POR :1;
        unsigned int PD :1;
        unsigned int TO :1;
        unsigned int RI :1;
        unsigned int CM :1;
    };
} RCONbits_t;
extern volatile RCONbits_t RCONbits __attribute__((address(0xFD0)));
# 35545 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char WDTCON __attribute__((address(0xFD1)));

__asm("WDTCON equ 0FD1h");


typedef union {
    struct {
        unsigned int SWDTEN :1;
        unsigned int ULPSINK :1;
        unsigned int ULPEN :1;
        unsigned int :1;
        unsigned int SRETEN :1;
        unsigned int ULPLVL :1;
        unsigned int :1;
        unsigned int REGSLP :1;
    };
    struct {
        unsigned int SWDTE :1;
    };
} WDTCONbits_t;
extern volatile WDTCONbits_t WDTCONbits __attribute__((address(0xFD1)));
# 35605 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char OSCCON2 __attribute__((address(0xFD2)));

__asm("OSCCON2 equ 0FD2h");


typedef union {
    struct {
        unsigned int MFIOSEL :1;
        unsigned int MFIOFS :1;
        unsigned int :1;
        unsigned int SOSCGO :1;
        unsigned int SOSCDRV :1;
        unsigned int :1;
        unsigned int SOSCRUN :1;
    };
    struct {
        unsigned int LVDL0 :1;
        unsigned int LVDL1 :1;
        unsigned int LVDL2 :1;
        unsigned int LVDL3 :1;
    };
} OSCCON2bits_t;
extern volatile OSCCON2bits_t OSCCON2bits __attribute__((address(0xFD2)));
# 35677 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char OSCCON __attribute__((address(0xFD3)));

__asm("OSCCON equ 0FD3h");


typedef union {
    struct {
        unsigned int SCS :2;
        unsigned int HFIOFS :1;
        unsigned int OSTS :1;
        unsigned int IRCF :3;
        unsigned int IDLEN :1;
    };
    struct {
        unsigned int SCS0 :1;
        unsigned int SCS1 :1;
        unsigned int :2;
        unsigned int IRCF0 :1;
        unsigned int IRCF1 :1;
        unsigned int IRCF2 :1;
    };
} OSCCONbits_t;
extern volatile OSCCONbits_t OSCCONbits __attribute__((address(0xFD3)));
# 35754 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char T0CON __attribute__((address(0xFD5)));

__asm("T0CON equ 0FD5h");


typedef union {
    struct {
        unsigned int T0PS :3;
        unsigned int PSA :1;
        unsigned int T0SE :1;
        unsigned int T0CS :1;
        unsigned int T08BIT :1;
        unsigned int TMR0ON :1;
    };
    struct {
        unsigned int T0PS0 :1;
        unsigned int T0PS1 :1;
        unsigned int T0PS2 :1;
    };
} T0CONbits_t;
extern volatile T0CONbits_t T0CONbits __attribute__((address(0xFD5)));
# 35824 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned short TMR0 __attribute__((address(0xFD6)));

__asm("TMR0 equ 0FD6h");




extern volatile unsigned char TMR0L __attribute__((address(0xFD6)));

__asm("TMR0L equ 0FD6h");


typedef union {
    struct {
        unsigned int TMR0L :8;
    };
} TMR0Lbits_t;
extern volatile TMR0Lbits_t TMR0Lbits __attribute__((address(0xFD6)));
# 35851 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TMR0H __attribute__((address(0xFD7)));

__asm("TMR0H equ 0FD7h");


typedef union {
    struct {
        unsigned int TMR0H :8;
    };
} TMR0Hbits_t;
extern volatile TMR0Hbits_t TMR0Hbits __attribute__((address(0xFD7)));
# 35871 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char STATUS __attribute__((address(0xFD8)));

__asm("STATUS equ 0FD8h");


typedef union {
    struct {
        unsigned int C :1;
        unsigned int DC :1;
        unsigned int Z :1;
        unsigned int OV :1;
        unsigned int N :1;
    };
    struct {
        unsigned int CARRY :1;
        unsigned int :1;
        unsigned int ZERO :1;
        unsigned int OVERFLOW :1;
        unsigned int NEGATIVE :1;
    };
} STATUSbits_t;
extern volatile STATUSbits_t STATUSbits __attribute__((address(0xFD8)));
# 35942 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned short FSR2 __attribute__((address(0xFD9)));

__asm("FSR2 equ 0FD9h");




extern volatile unsigned char FSR2L __attribute__((address(0xFD9)));

__asm("FSR2L equ 0FD9h");


typedef union {
    struct {
        unsigned int FSR2L :8;
    };
} FSR2Lbits_t;
extern volatile FSR2Lbits_t FSR2Lbits __attribute__((address(0xFD9)));
# 35969 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char FSR2H __attribute__((address(0xFDA)));

__asm("FSR2H equ 0FDAh");


typedef union {
    struct {
        unsigned int FSR2H :4;
    };
} FSR2Hbits_t;
extern volatile FSR2Hbits_t FSR2Hbits __attribute__((address(0xFDA)));
# 35989 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char PLUSW2 __attribute__((address(0xFDB)));

__asm("PLUSW2 equ 0FDBh");


typedef union {
    struct {
        unsigned int PLUSW2 :8;
    };
} PLUSW2bits_t;
extern volatile PLUSW2bits_t PLUSW2bits __attribute__((address(0xFDB)));
# 36009 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char PREINC2 __attribute__((address(0xFDC)));

__asm("PREINC2 equ 0FDCh");


typedef union {
    struct {
        unsigned int PREINC2 :8;
    };
} PREINC2bits_t;
extern volatile PREINC2bits_t PREINC2bits __attribute__((address(0xFDC)));
# 36029 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char POSTDEC2 __attribute__((address(0xFDD)));

__asm("POSTDEC2 equ 0FDDh");


typedef union {
    struct {
        unsigned int POSTDEC2 :8;
    };
} POSTDEC2bits_t;
extern volatile POSTDEC2bits_t POSTDEC2bits __attribute__((address(0xFDD)));
# 36049 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char POSTINC2 __attribute__((address(0xFDE)));

__asm("POSTINC2 equ 0FDEh");


typedef union {
    struct {
        unsigned int POSTINC2 :8;
    };
} POSTINC2bits_t;
extern volatile POSTINC2bits_t POSTINC2bits __attribute__((address(0xFDE)));
# 36069 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char INDF2 __attribute__((address(0xFDF)));

__asm("INDF2 equ 0FDFh");


typedef union {
    struct {
        unsigned int INDF2 :8;
    };
} INDF2bits_t;
extern volatile INDF2bits_t INDF2bits __attribute__((address(0xFDF)));
# 36089 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char BSR __attribute__((address(0xFE0)));

__asm("BSR equ 0FE0h");


typedef union {
    struct {
        unsigned int BSR :4;
    };
} BSRbits_t;
extern volatile BSRbits_t BSRbits __attribute__((address(0xFE0)));
# 36109 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned short FSR1 __attribute__((address(0xFE1)));

__asm("FSR1 equ 0FE1h");




extern volatile unsigned char FSR1L __attribute__((address(0xFE1)));

__asm("FSR1L equ 0FE1h");


typedef union {
    struct {
        unsigned int FSR1L :8;
    };
} FSR1Lbits_t;
extern volatile FSR1Lbits_t FSR1Lbits __attribute__((address(0xFE1)));
# 36136 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char FSR1H __attribute__((address(0xFE2)));

__asm("FSR1H equ 0FE2h");


typedef union {
    struct {
        unsigned int FSR1H :4;
    };
} FSR1Hbits_t;
extern volatile FSR1Hbits_t FSR1Hbits __attribute__((address(0xFE2)));
# 36156 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char PLUSW1 __attribute__((address(0xFE3)));

__asm("PLUSW1 equ 0FE3h");


typedef union {
    struct {
        unsigned int PLUSW1 :8;
    };
} PLUSW1bits_t;
extern volatile PLUSW1bits_t PLUSW1bits __attribute__((address(0xFE3)));
# 36176 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char PREINC1 __attribute__((address(0xFE4)));

__asm("PREINC1 equ 0FE4h");


typedef union {
    struct {
        unsigned int PREINC1 :8;
    };
} PREINC1bits_t;
extern volatile PREINC1bits_t PREINC1bits __attribute__((address(0xFE4)));
# 36196 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char POSTDEC1 __attribute__((address(0xFE5)));

__asm("POSTDEC1 equ 0FE5h");


typedef union {
    struct {
        unsigned int POSTDEC1 :8;
    };
} POSTDEC1bits_t;
extern volatile POSTDEC1bits_t POSTDEC1bits __attribute__((address(0xFE5)));
# 36216 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char POSTINC1 __attribute__((address(0xFE6)));

__asm("POSTINC1 equ 0FE6h");


typedef union {
    struct {
        unsigned int POSTINC1 :8;
    };
} POSTINC1bits_t;
extern volatile POSTINC1bits_t POSTINC1bits __attribute__((address(0xFE6)));
# 36236 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char INDF1 __attribute__((address(0xFE7)));

__asm("INDF1 equ 0FE7h");


typedef union {
    struct {
        unsigned int INDF1 :8;
    };
} INDF1bits_t;
extern volatile INDF1bits_t INDF1bits __attribute__((address(0xFE7)));
# 36256 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char WREG __attribute__((address(0xFE8)));

__asm("WREG equ 0FE8h");


typedef union {
    struct {
        unsigned int WREG :8;
    };
} WREGbits_t;
extern volatile WREGbits_t WREGbits __attribute__((address(0xFE8)));
# 36276 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned short FSR0 __attribute__((address(0xFE9)));

__asm("FSR0 equ 0FE9h");




extern volatile unsigned char FSR0L __attribute__((address(0xFE9)));

__asm("FSR0L equ 0FE9h");


typedef union {
    struct {
        unsigned int FSR0L :8;
    };
} FSR0Lbits_t;
extern volatile FSR0Lbits_t FSR0Lbits __attribute__((address(0xFE9)));
# 36303 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char FSR0H __attribute__((address(0xFEA)));

__asm("FSR0H equ 0FEAh");


typedef union {
    struct {
        unsigned int FSR0H :4;
    };
} FSR0Hbits_t;
extern volatile FSR0Hbits_t FSR0Hbits __attribute__((address(0xFEA)));
# 36323 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char PLUSW0 __attribute__((address(0xFEB)));

__asm("PLUSW0 equ 0FEBh");


typedef union {
    struct {
        unsigned int PLUSW0 :8;
    };
} PLUSW0bits_t;
extern volatile PLUSW0bits_t PLUSW0bits __attribute__((address(0xFEB)));
# 36343 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char PREINC0 __attribute__((address(0xFEC)));

__asm("PREINC0 equ 0FECh");


typedef union {
    struct {
        unsigned int PREINC0 :8;
    };
} PREINC0bits_t;
extern volatile PREINC0bits_t PREINC0bits __attribute__((address(0xFEC)));
# 36363 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char POSTDEC0 __attribute__((address(0xFED)));

__asm("POSTDEC0 equ 0FEDh");


typedef union {
    struct {
        unsigned int POSTDEC0 :8;
    };
} POSTDEC0bits_t;
extern volatile POSTDEC0bits_t POSTDEC0bits __attribute__((address(0xFED)));
# 36383 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char POSTINC0 __attribute__((address(0xFEE)));

__asm("POSTINC0 equ 0FEEh");


typedef union {
    struct {
        unsigned int POSTINC0 :8;
    };
} POSTINC0bits_t;
extern volatile POSTINC0bits_t POSTINC0bits __attribute__((address(0xFEE)));
# 36403 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char INDF0 __attribute__((address(0xFEF)));

__asm("INDF0 equ 0FEFh");


typedef union {
    struct {
        unsigned int INDF0 :8;
    };
} INDF0bits_t;
extern volatile INDF0bits_t INDF0bits __attribute__((address(0xFEF)));
# 36423 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char INTCON3 __attribute__((address(0xFF0)));

__asm("INTCON3 equ 0FF0h");


typedef union {
    struct {
        unsigned int INT1IF :1;
        unsigned int INT2IF :1;
        unsigned int INT3IF :1;
        unsigned int INT1IE :1;
        unsigned int INT2IE :1;
        unsigned int INT3IE :1;
        unsigned int INT1IP :1;
        unsigned int INT2IP :1;
    };
    struct {
        unsigned int INT1F :1;
        unsigned int INT2F :1;
        unsigned int INT3F :1;
        unsigned int INT1E :1;
        unsigned int INT2E :1;
        unsigned int INT3E :1;
        unsigned int INT1P :1;
        unsigned int INT2P :1;
    };
} INTCON3bits_t;
extern volatile INTCON3bits_t INTCON3bits __attribute__((address(0xFF0)));
# 36535 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char INTCON2 __attribute__((address(0xFF1)));

__asm("INTCON2 equ 0FF1h");


typedef union {
    struct {
        unsigned int :7;
        unsigned int NOT_RBPU :1;
    };
    struct {
        unsigned int RBIP :1;
        unsigned int INT3IP :1;
        unsigned int TMR0IP :1;
        unsigned int INTEDG3 :1;
        unsigned int INTEDG2 :1;
        unsigned int INTEDG1 :1;
        unsigned int INTEDG0 :1;
        unsigned int nRBPU :1;
    };
    struct {
        unsigned int :1;
        unsigned int INT3P :1;
        unsigned int T0IP :1;
        unsigned int :4;
        unsigned int RBPU :1;
    };
} INTCON2bits_t;
extern volatile INTCON2bits_t INTCON2bits __attribute__((address(0xFF1)));
# 36628 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char INTCON __attribute__((address(0xFF2)));

__asm("INTCON equ 0FF2h");


extern volatile unsigned char INTCON1 __attribute__((address(0xFF2)));

__asm("INTCON1 equ 0FF2h");


typedef union {
    struct {
        unsigned int RBIF :1;
        unsigned int INT0IF :1;
        unsigned int TMR0IF :1;
        unsigned int RBIE :1;
        unsigned int INT0IE :1;
        unsigned int TMR0IE :1;
        unsigned int PEIE_GIEL :1;
        unsigned int GIE_GIEH :1;
    };
    struct {
        unsigned int :1;
        unsigned int INT0F :1;
        unsigned int T0IF :1;
        unsigned int :1;
        unsigned int INT0E :1;
        unsigned int T0IE :1;
        unsigned int PEIE :1;
        unsigned int GIE :1;
    };
    struct {
        unsigned int :6;
        unsigned int GIEL :1;
        unsigned int GIEH :1;
    };
} INTCONbits_t;
extern volatile INTCONbits_t INTCONbits __attribute__((address(0xFF2)));
# 36748 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
typedef union {
    struct {
        unsigned int RBIF :1;
        unsigned int INT0IF :1;
        unsigned int TMR0IF :1;
        unsigned int RBIE :1;
        unsigned int INT0IE :1;
        unsigned int TMR0IE :1;
        unsigned int PEIE_GIEL :1;
        unsigned int GIE_GIEH :1;
    };
    struct {
        unsigned int :1;
        unsigned int INT0F :1;
        unsigned int T0IF :1;
        unsigned int :1;
        unsigned int INT0E :1;
        unsigned int T0IE :1;
        unsigned int PEIE :1;
        unsigned int GIE :1;
    };
    struct {
        unsigned int :6;
        unsigned int GIEL :1;
        unsigned int GIEH :1;
    };
} INTCON1bits_t;
extern volatile INTCON1bits_t INTCON1bits __attribute__((address(0xFF2)));
# 36860 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned short PROD __attribute__((address(0xFF3)));

__asm("PROD equ 0FF3h");




extern volatile unsigned char PRODL __attribute__((address(0xFF3)));

__asm("PRODL equ 0FF3h");


typedef union {
    struct {
        unsigned int PRODL :8;
    };
} PRODLbits_t;
extern volatile PRODLbits_t PRODLbits __attribute__((address(0xFF3)));
# 36887 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char PRODH __attribute__((address(0xFF4)));

__asm("PRODH equ 0FF4h");


typedef union {
    struct {
        unsigned int PRODH :8;
    };
} PRODHbits_t;
extern volatile PRODHbits_t PRODHbits __attribute__((address(0xFF4)));
# 36907 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TABLAT __attribute__((address(0xFF5)));

__asm("TABLAT equ 0FF5h");


typedef union {
    struct {
        unsigned int TABLAT :8;
    };
} TABLATbits_t;
extern volatile TABLATbits_t TABLATbits __attribute__((address(0xFF5)));
# 36928 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile __uint24 TBLPTR __attribute__((address(0xFF6)));


__asm("TBLPTR equ 0FF6h");




extern volatile unsigned char TBLPTRL __attribute__((address(0xFF6)));

__asm("TBLPTRL equ 0FF6h");


typedef union {
    struct {
        unsigned int TBLPTRL :8;
    };
} TBLPTRLbits_t;
extern volatile TBLPTRLbits_t TBLPTRLbits __attribute__((address(0xFF6)));
# 36956 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TBLPTRH __attribute__((address(0xFF7)));

__asm("TBLPTRH equ 0FF7h");


typedef union {
    struct {
        unsigned int TBLPTRH :8;
    };
} TBLPTRHbits_t;
extern volatile TBLPTRHbits_t TBLPTRHbits __attribute__((address(0xFF7)));
# 36976 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TBLPTRU __attribute__((address(0xFF8)));

__asm("TBLPTRU equ 0FF8h");


typedef union {
    struct {
        unsigned int TBLPTRU :5;
    };
} TBLPTRUbits_t;
extern volatile TBLPTRUbits_t TBLPTRUbits __attribute__((address(0xFF8)));
# 36997 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile __uint24 PCLAT __attribute__((address(0xFF9)));


__asm("PCLAT equ 0FF9h");



extern volatile __uint24 PC __attribute__((address(0xFF9)));


__asm("PC equ 0FF9h");




extern volatile unsigned char PCL __attribute__((address(0xFF9)));

__asm("PCL equ 0FF9h");


typedef union {
    struct {
        unsigned int PCL :8;
    };
} PCLbits_t;
extern volatile PCLbits_t PCLbits __attribute__((address(0xFF9)));
# 37032 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char PCLATH __attribute__((address(0xFFA)));

__asm("PCLATH equ 0FFAh");


typedef union {
    struct {
        unsigned int PCH :8;
    };
} PCLATHbits_t;
extern volatile PCLATHbits_t PCLATHbits __attribute__((address(0xFFA)));
# 37052 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char PCLATU __attribute__((address(0xFFB)));

__asm("PCLATU equ 0FFBh");


typedef union {
    struct {
        unsigned int PCU :5;
    };
} PCLATUbits_t;
extern volatile PCLATUbits_t PCLATUbits __attribute__((address(0xFFB)));
# 37072 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char STKPTR __attribute__((address(0xFFC)));

__asm("STKPTR equ 0FFCh");


typedef union {
    struct {
        unsigned int STKPTR :5;
        unsigned int :1;
        unsigned int STKUNF :1;
        unsigned int STKFUL :1;
    };
    struct {
        unsigned int SP0 :1;
        unsigned int SP1 :1;
        unsigned int SP2 :1;
        unsigned int SP3 :1;
        unsigned int SP4 :1;
        unsigned int :2;
        unsigned int STKOVF :1;
    };
} STKPTRbits_t;
extern volatile STKPTRbits_t STKPTRbits __attribute__((address(0xFFC)));
# 37145 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile __uint24 TOS __attribute__((address(0xFFD)));


__asm("TOS equ 0FFDh");




extern volatile unsigned char TOSL __attribute__((address(0xFFD)));

__asm("TOSL equ 0FFDh");


typedef union {
    struct {
        unsigned int TOSL :8;
    };
} TOSLbits_t;
extern volatile TOSLbits_t TOSLbits __attribute__((address(0xFFD)));
# 37173 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TOSH __attribute__((address(0xFFE)));

__asm("TOSH equ 0FFEh");


typedef union {
    struct {
        unsigned int TOSH :8;
    };
} TOSHbits_t;
extern volatile TOSHbits_t TOSHbits __attribute__((address(0xFFE)));
# 37193 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile unsigned char TOSU __attribute__((address(0xFFF)));

__asm("TOSU equ 0FFFh");


typedef union {
    struct {
        unsigned int TOSU :5;
    };
} TOSUbits_t;
extern volatile TOSUbits_t TOSUbits __attribute__((address(0xFFF)));
# 37214 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern const unsigned char _CONFIGs[14] __attribute__((address(0x300000)));

extern const unsigned char _CONFIG1L __attribute__((address(0x300000)));

typedef union {
    struct {
        unsigned int RETEN :1;
        unsigned int :1;
        unsigned int INTOSCSEL :1;
        unsigned int SOSCSEL :2;
        unsigned int :1;
        unsigned int XINST :1;
    };
} _CONFIG1Lbits_t;
extern const _CONFIG1Lbits_t _CONFIG1Lbits __attribute__((address(0x300000)));


extern const unsigned char _CONFIG1H __attribute__((address(0x300001)));

typedef union {
    struct {
        unsigned int FOSC :4;
        unsigned int PLLCFG :1;
        unsigned int :1;
        unsigned int FCMEN :1;
        unsigned int IESO :1;
    };
} _CONFIG1Hbits_t;
extern const _CONFIG1Hbits_t _CONFIG1Hbits __attribute__((address(0x300001)));


extern const unsigned char _CONFIG2L __attribute__((address(0x300002)));

typedef union {
    struct {
        unsigned int PWRTEN :1;
        unsigned int BOREN :2;
        unsigned int BORV :2;
        unsigned int BORPWR :2;
    };
} _CONFIG2Lbits_t;
extern const _CONFIG2Lbits_t _CONFIG2Lbits __attribute__((address(0x300002)));


extern const unsigned char _CONFIG2H __attribute__((address(0x300003)));

typedef union {
    struct {
        unsigned int WDTEN :2;
        unsigned int WDTPS :5;
    };
} _CONFIG2Hbits_t;
extern const _CONFIG2Hbits_t _CONFIG2Hbits __attribute__((address(0x300003)));


extern const unsigned char _CONFIG3H __attribute__((address(0x300005)));

typedef union {
    struct {
        unsigned int CANMX :1;
        unsigned int :2;
        unsigned int MSSPMSK :1;
        unsigned int :3;
        unsigned int MCLRE :1;
    };
} _CONFIG3Hbits_t;
extern const _CONFIG3Hbits_t _CONFIG3Hbits __attribute__((address(0x300005)));


extern const unsigned char _CONFIG4L __attribute__((address(0x300006)));

typedef union {
    struct {
        unsigned int STVREN :1;
        unsigned int :3;
        unsigned int BBSIZ :1;
        unsigned int :2;
        unsigned int _DEBUG :1;
    };
} _CONFIG4Lbits_t;
extern const _CONFIG4Lbits_t _CONFIG4Lbits __attribute__((address(0x300006)));


extern const unsigned char _CONFIG5L __attribute__((address(0x300008)));

typedef union {
    struct {
        unsigned int CP0 :1;
        unsigned int CP1 :1;
        unsigned int CP2 :1;
        unsigned int CP3 :1;
    };
} _CONFIG5Lbits_t;
extern const _CONFIG5Lbits_t _CONFIG5Lbits __attribute__((address(0x300008)));


extern const unsigned char _CONFIG5H __attribute__((address(0x300009)));

typedef union {
    struct {
        unsigned int :6;
        unsigned int CPB :1;
        unsigned int CPD :1;
    };
} _CONFIG5Hbits_t;
extern const _CONFIG5Hbits_t _CONFIG5Hbits __attribute__((address(0x300009)));


extern const unsigned char _CONFIG6L __attribute__((address(0x30000A)));

typedef union {
    struct {
        unsigned int WRT0 :1;
        unsigned int WRT1 :1;
        unsigned int WRT2 :1;
        unsigned int WRT3 :1;
    };
} _CONFIG6Lbits_t;
extern const _CONFIG6Lbits_t _CONFIG6Lbits __attribute__((address(0x30000A)));


extern const unsigned char _CONFIG6H __attribute__((address(0x30000B)));

typedef union {
    struct {
        unsigned int :5;
        unsigned int WRTC :1;
        unsigned int WRTB :1;
        unsigned int WRTD :1;
    };
} _CONFIG6Hbits_t;
extern const _CONFIG6Hbits_t _CONFIG6Hbits __attribute__((address(0x30000B)));


extern const unsigned char _CONFIG7L __attribute__((address(0x30000C)));

typedef union {
    struct {
        unsigned int EBTR0 :1;
        unsigned int EBTR1 :1;
        unsigned int EBTR2 :1;
        unsigned int EBTR3 :1;
    };
} _CONFIG7Lbits_t;
extern const _CONFIG7Lbits_t _CONFIG7Lbits __attribute__((address(0x30000C)));


extern const unsigned char _CONFIG7H __attribute__((address(0x30000D)));

typedef union {
    struct {
        unsigned int :6;
        unsigned int EBTRB :1;
    };
} _CONFIG7Hbits_t;
extern const _CONFIG7Hbits_t _CONFIG7Hbits __attribute__((address(0x30000D)));

extern const unsigned char _IDLOCs[8] __attribute__((address(0x200000)));

extern const unsigned char _IDLOC0 __attribute__((address(0x200000)));


extern const unsigned char _IDLOC1 __attribute__((address(0x200001)));


extern const unsigned char _IDLOC2 __attribute__((address(0x200002)));


extern const unsigned char _IDLOC3 __attribute__((address(0x200003)));


extern const unsigned char _IDLOC4 __attribute__((address(0x200004)));


extern const unsigned char _IDLOC5 __attribute__((address(0x200005)));


extern const unsigned char _IDLOC6 __attribute__((address(0x200006)));


extern const unsigned char _IDLOC7 __attribute__((address(0x200007)));




extern const unsigned short _DEVID __attribute__((address(0x3FFFFE)));
# 37413 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include\\proc/pic18f26k80.h" 3
extern volatile __bit ABDEN1 __attribute__((address(0x7D38)));


extern volatile __bit ABDEN2 __attribute__((address(0x7DC8)));


extern volatile __bit ABDOVF1 __attribute__((address(0x7D3F)));


extern volatile __bit ABDOVF2 __attribute__((address(0x7DCF)));


extern volatile __bit ACKDT __attribute__((address(0x7E2D)));


extern volatile __bit ACKEN __attribute__((address(0x7E2C)));


extern volatile __bit ACKSTAT __attribute__((address(0x7E2E)));


extern volatile __bit ACQT0 __attribute__((address(0x7E03)));


extern volatile __bit ACQT1 __attribute__((address(0x7E04)));


extern volatile __bit ACQT2 __attribute__((address(0x7E05)));


extern volatile __bit ADCMD __attribute__((address(0x7AC5)));


extern volatile __bit ADCS0 __attribute__((address(0x7E00)));


extern volatile __bit ADCS1 __attribute__((address(0x7E01)));


extern volatile __bit ADCS2 __attribute__((address(0x7E02)));


extern volatile __bit ADDEN1 __attribute__((address(0x7D5B)));


extern volatile __bit ADDEN2 __attribute__((address(0x7D33)));


extern volatile __bit ADEN __attribute__((address(0x7D5B)));


extern volatile __bit ADFM __attribute__((address(0x7E07)));


extern volatile __bit ADIE __attribute__((address(0x7CEE)));


extern volatile __bit ADIF __attribute__((address(0x7CF6)));


extern volatile __bit ADIP __attribute__((address(0x7CFE)));


extern volatile __bit ADMSK1 __attribute__((address(0x7E29)));


extern volatile __bit ADMSK2 __attribute__((address(0x7E2A)));


extern volatile __bit ADMSK3 __attribute__((address(0x7E2B)));


extern volatile __bit ADMSK4 __attribute__((address(0x7E2C)));


extern volatile __bit ADMSK5 __attribute__((address(0x7E2D)));


extern volatile __bit ADON __attribute__((address(0x7E10)));


extern volatile __bit ANSEL0 __attribute__((address(0x7AE8)));


extern volatile __bit ANSEL1 __attribute__((address(0x7AE9)));


extern volatile __bit ANSEL10 __attribute__((address(0x7AE2)));


extern volatile __bit ANSEL2 __attribute__((address(0x7AEA)));


extern volatile __bit ANSEL3 __attribute__((address(0x7AEB)));


extern volatile __bit ANSEL4 __attribute__((address(0x7AEC)));


extern volatile __bit ANSEL8 __attribute__((address(0x7AE0)));


extern volatile __bit ANSEL9 __attribute__((address(0x7AE1)));


extern volatile __bit B0D00 __attribute__((address(0x7430)));


extern volatile __bit B0D01 __attribute__((address(0x7431)));


extern volatile __bit B0D02 __attribute__((address(0x7432)));


extern volatile __bit B0D03 __attribute__((address(0x7433)));


extern volatile __bit B0D04 __attribute__((address(0x7434)));


extern volatile __bit B0D05 __attribute__((address(0x7435)));


extern volatile __bit B0D06 __attribute__((address(0x7436)));


extern volatile __bit B0D07 __attribute__((address(0x7437)));


extern volatile __bit B0D10 __attribute__((address(0x7438)));


extern volatile __bit B0D11 __attribute__((address(0x7439)));


extern volatile __bit B0D12 __attribute__((address(0x743A)));


extern volatile __bit B0D13 __attribute__((address(0x743B)));


extern volatile __bit B0D14 __attribute__((address(0x743C)));


extern volatile __bit B0D15 __attribute__((address(0x743D)));


extern volatile __bit B0D16 __attribute__((address(0x743E)));


extern volatile __bit B0D17 __attribute__((address(0x743F)));


extern volatile __bit B0D20 __attribute__((address(0x7440)));


extern volatile __bit B0D21 __attribute__((address(0x7441)));


extern volatile __bit B0D22 __attribute__((address(0x7442)));


extern volatile __bit B0D23 __attribute__((address(0x7443)));


extern volatile __bit B0D24 __attribute__((address(0x7444)));


extern volatile __bit B0D25 __attribute__((address(0x7445)));


extern volatile __bit B0D26 __attribute__((address(0x7446)));


extern volatile __bit B0D27 __attribute__((address(0x7447)));


extern volatile __bit B0D30 __attribute__((address(0x7448)));


extern volatile __bit B0D31 __attribute__((address(0x7449)));


extern volatile __bit B0D32 __attribute__((address(0x744A)));


extern volatile __bit B0D33 __attribute__((address(0x744B)));


extern volatile __bit B0D34 __attribute__((address(0x744C)));


extern volatile __bit B0D35 __attribute__((address(0x744D)));


extern volatile __bit B0D36 __attribute__((address(0x744E)));


extern volatile __bit B0D37 __attribute__((address(0x744F)));


extern volatile __bit B0D40 __attribute__((address(0x7450)));


extern volatile __bit B0D41 __attribute__((address(0x7451)));


extern volatile __bit B0D42 __attribute__((address(0x7452)));


extern volatile __bit B0D43 __attribute__((address(0x7453)));


extern volatile __bit B0D44 __attribute__((address(0x7454)));


extern volatile __bit B0D45 __attribute__((address(0x7455)));


extern volatile __bit B0D46 __attribute__((address(0x7456)));


extern volatile __bit B0D47 __attribute__((address(0x7457)));


extern volatile __bit B0D50 __attribute__((address(0x7458)));


extern volatile __bit B0D51 __attribute__((address(0x7459)));


extern volatile __bit B0D52 __attribute__((address(0x745A)));


extern volatile __bit B0D53 __attribute__((address(0x745B)));


extern volatile __bit B0D54 __attribute__((address(0x745C)));


extern volatile __bit B0D55 __attribute__((address(0x745D)));


extern volatile __bit B0D56 __attribute__((address(0x745E)));


extern volatile __bit B0D57 __attribute__((address(0x745F)));


extern volatile __bit B0D60 __attribute__((address(0x7460)));


extern volatile __bit B0D61 __attribute__((address(0x7461)));


extern volatile __bit B0D62 __attribute__((address(0x7462)));


extern volatile __bit B0D63 __attribute__((address(0x7463)));


extern volatile __bit B0D64 __attribute__((address(0x7464)));


extern volatile __bit B0D65 __attribute__((address(0x7465)));


extern volatile __bit B0D66 __attribute__((address(0x7466)));


extern volatile __bit B0D67 __attribute__((address(0x7467)));


extern volatile __bit B0D70 __attribute__((address(0x7468)));


extern volatile __bit B0D71 __attribute__((address(0x7469)));


extern volatile __bit B0D72 __attribute__((address(0x746A)));


extern volatile __bit B0D73 __attribute__((address(0x746B)));


extern volatile __bit B0D74 __attribute__((address(0x746C)));


extern volatile __bit B0D75 __attribute__((address(0x746D)));


extern volatile __bit B0D76 __attribute__((address(0x746E)));


extern volatile __bit B0D77 __attribute__((address(0x746F)));


extern volatile __bit B0DLC0 __attribute__((address(0x7428)));


extern volatile __bit B0DLC1 __attribute__((address(0x7429)));


extern volatile __bit B0DLC2 __attribute__((address(0x742A)));


extern volatile __bit B0DLC3 __attribute__((address(0x742B)));


extern volatile __bit B0EID0 __attribute__((address(0x7420)));


extern volatile __bit B0EID1 __attribute__((address(0x7421)));


extern volatile __bit B0EID10 __attribute__((address(0x741A)));


extern volatile __bit B0EID11 __attribute__((address(0x741B)));


extern volatile __bit B0EID12 __attribute__((address(0x741C)));


extern volatile __bit B0EID13 __attribute__((address(0x741D)));


extern volatile __bit B0EID14 __attribute__((address(0x741E)));


extern volatile __bit B0EID15 __attribute__((address(0x741F)));


extern volatile __bit B0EID16 __attribute__((address(0x7410)));


extern volatile __bit B0EID17 __attribute__((address(0x7411)));


extern volatile __bit B0EID2 __attribute__((address(0x7422)));


extern volatile __bit B0EID3 __attribute__((address(0x7423)));


extern volatile __bit B0EID4 __attribute__((address(0x7424)));


extern volatile __bit B0EID5 __attribute__((address(0x7425)));


extern volatile __bit B0EID6 __attribute__((address(0x7426)));


extern volatile __bit B0EID7 __attribute__((address(0x7427)));


extern volatile __bit B0EID8 __attribute__((address(0x7418)));


extern volatile __bit B0EID9 __attribute__((address(0x7419)));


extern volatile __bit B0EXID __attribute__((address(0x7413)));


extern volatile __bit B0FILHIT0 __attribute__((address(0x7400)));


extern volatile __bit B0FILHIT1 __attribute__((address(0x7401)));


extern volatile __bit B0FILHIT2 __attribute__((address(0x7402)));


extern volatile __bit B0FILHIT3 __attribute__((address(0x7403)));


extern volatile __bit B0FILHIT4 __attribute__((address(0x7404)));


extern volatile __bit B0IE __attribute__((address(0x73F2)));


extern volatile __bit B0RB0 __attribute__((address(0x742C)));


extern volatile __bit B0RB1 __attribute__((address(0x742D)));


extern volatile __bit B0RTREN __attribute__((address(0x7402)));


extern volatile __bit B0RTRRO __attribute__((address(0x7405)));


extern volatile __bit B0RXFUL __attribute__((address(0x7407)));


extern volatile __bit B0RXM1 __attribute__((address(0x7406)));


extern volatile __bit B0RXRTR __attribute__((address(0x742E)));


extern volatile __bit B0SID0 __attribute__((address(0x7415)));


extern volatile __bit B0SID1 __attribute__((address(0x7416)));


extern volatile __bit B0SID10 __attribute__((address(0x740F)));


extern volatile __bit B0SID2 __attribute__((address(0x7417)));


extern volatile __bit B0SID3 __attribute__((address(0x7408)));


extern volatile __bit B0SID4 __attribute__((address(0x7409)));


extern volatile __bit B0SID5 __attribute__((address(0x740A)));


extern volatile __bit B0SID6 __attribute__((address(0x740B)));


extern volatile __bit B0SID7 __attribute__((address(0x740C)));


extern volatile __bit B0SID8 __attribute__((address(0x740D)));


extern volatile __bit B0SID9 __attribute__((address(0x740E)));


extern volatile __bit B0SRR __attribute__((address(0x7414)));


extern volatile __bit B0TXABT __attribute__((address(0x7406)));


extern volatile __bit B0TXB3IF __attribute__((address(0x7407)));


extern volatile __bit B0TXEN __attribute__((address(0x73EA)));


extern volatile __bit B0TXERR __attribute__((address(0x7404)));


extern volatile __bit B0TXLARB __attribute__((address(0x7405)));


extern volatile __bit B0TXPRI0 __attribute__((address(0x7400)));


extern volatile __bit B0TXPRI1 __attribute__((address(0x7401)));


extern volatile __bit B0TXREQ __attribute__((address(0x7403)));


extern volatile __bit B1D00 __attribute__((address(0x74B0)));


extern volatile __bit B1D01 __attribute__((address(0x74B1)));


extern volatile __bit B1D02 __attribute__((address(0x74B2)));


extern volatile __bit B1D03 __attribute__((address(0x74B3)));


extern volatile __bit B1D04 __attribute__((address(0x74B4)));


extern volatile __bit B1D05 __attribute__((address(0x74B5)));


extern volatile __bit B1D06 __attribute__((address(0x74B6)));


extern volatile __bit B1D07 __attribute__((address(0x74B7)));


extern volatile __bit B1D10 __attribute__((address(0x74B8)));


extern volatile __bit B1D11 __attribute__((address(0x74B9)));


extern volatile __bit B1D12 __attribute__((address(0x74BA)));


extern volatile __bit B1D13 __attribute__((address(0x74BB)));


extern volatile __bit B1D14 __attribute__((address(0x74BC)));


extern volatile __bit B1D15 __attribute__((address(0x74BD)));


extern volatile __bit B1D16 __attribute__((address(0x74BE)));


extern volatile __bit B1D17 __attribute__((address(0x74BF)));


extern volatile __bit B1D20 __attribute__((address(0x74C0)));


extern volatile __bit B1D21 __attribute__((address(0x74C1)));


extern volatile __bit B1D22 __attribute__((address(0x74C2)));


extern volatile __bit B1D23 __attribute__((address(0x74C3)));


extern volatile __bit B1D24 __attribute__((address(0x74C4)));


extern volatile __bit B1D25 __attribute__((address(0x74C5)));


extern volatile __bit B1D26 __attribute__((address(0x74C6)));


extern volatile __bit B1D27 __attribute__((address(0x74C7)));


extern volatile __bit B1D30 __attribute__((address(0x74C8)));


extern volatile __bit B1D31 __attribute__((address(0x74C9)));


extern volatile __bit B1D32 __attribute__((address(0x74CA)));


extern volatile __bit B1D33 __attribute__((address(0x74CB)));


extern volatile __bit B1D34 __attribute__((address(0x74CC)));


extern volatile __bit B1D35 __attribute__((address(0x74CD)));


extern volatile __bit B1D36 __attribute__((address(0x74CE)));


extern volatile __bit B1D37 __attribute__((address(0x74CF)));


extern volatile __bit B1D40 __attribute__((address(0x74D0)));


extern volatile __bit B1D41 __attribute__((address(0x74D1)));


extern volatile __bit B1D42 __attribute__((address(0x74D2)));


extern volatile __bit B1D43 __attribute__((address(0x74D3)));


extern volatile __bit B1D44 __attribute__((address(0x74D4)));


extern volatile __bit B1D45 __attribute__((address(0x74D5)));


extern volatile __bit B1D46 __attribute__((address(0x74D6)));


extern volatile __bit B1D47 __attribute__((address(0x74D7)));


extern volatile __bit B1D50 __attribute__((address(0x74D8)));


extern volatile __bit B1D51 __attribute__((address(0x74D9)));


extern volatile __bit B1D52 __attribute__((address(0x74DA)));


extern volatile __bit B1D53 __attribute__((address(0x74DB)));


extern volatile __bit B1D54 __attribute__((address(0x74DC)));


extern volatile __bit B1D55 __attribute__((address(0x74DD)));


extern volatile __bit B1D56 __attribute__((address(0x74DE)));


extern volatile __bit B1D57 __attribute__((address(0x74DF)));


extern volatile __bit B1D60 __attribute__((address(0x74E0)));


extern volatile __bit B1D61 __attribute__((address(0x74E1)));


extern volatile __bit B1D62 __attribute__((address(0x74E2)));


extern volatile __bit B1D63 __attribute__((address(0x74E3)));


extern volatile __bit B1D64 __attribute__((address(0x74E4)));


extern volatile __bit B1D65 __attribute__((address(0x74E5)));


extern volatile __bit B1D66 __attribute__((address(0x74E6)));


extern volatile __bit B1D67 __attribute__((address(0x74E7)));


extern volatile __bit B1D70 __attribute__((address(0x74E8)));


extern volatile __bit B1D71 __attribute__((address(0x74E9)));


extern volatile __bit B1D72 __attribute__((address(0x74EA)));


extern volatile __bit B1D73 __attribute__((address(0x74EB)));


extern volatile __bit B1D74 __attribute__((address(0x74EC)));


extern volatile __bit B1D75 __attribute__((address(0x74ED)));


extern volatile __bit B1D76 __attribute__((address(0x74EE)));


extern volatile __bit B1D77 __attribute__((address(0x74EF)));


extern volatile __bit B1DLC0 __attribute__((address(0x74A8)));


extern volatile __bit B1DLC1 __attribute__((address(0x74A9)));


extern volatile __bit B1DLC2 __attribute__((address(0x74AA)));


extern volatile __bit B1DLC3 __attribute__((address(0x74AB)));


extern volatile __bit B1EID0 __attribute__((address(0x74A0)));


extern volatile __bit B1EID1 __attribute__((address(0x74A1)));


extern volatile __bit B1EID10 __attribute__((address(0x749A)));


extern volatile __bit B1EID11 __attribute__((address(0x749B)));


extern volatile __bit B1EID12 __attribute__((address(0x749C)));


extern volatile __bit B1EID13 __attribute__((address(0x749D)));


extern volatile __bit B1EID14 __attribute__((address(0x749E)));


extern volatile __bit B1EID15 __attribute__((address(0x749F)));


extern volatile __bit B1EID16 __attribute__((address(0x7490)));


extern volatile __bit B1EID17 __attribute__((address(0x7491)));


extern volatile __bit B1EID2 __attribute__((address(0x74A2)));


extern volatile __bit B1EID3 __attribute__((address(0x74A3)));


extern volatile __bit B1EID4 __attribute__((address(0x74A4)));


extern volatile __bit B1EID5 __attribute__((address(0x74A5)));


extern volatile __bit B1EID6 __attribute__((address(0x74A6)));


extern volatile __bit B1EID7 __attribute__((address(0x74A7)));


extern volatile __bit B1EID8 __attribute__((address(0x7498)));


extern volatile __bit B1EID9 __attribute__((address(0x7499)));


extern volatile __bit B1EXID __attribute__((address(0x7493)));


extern volatile __bit B1FILHIT0 __attribute__((address(0x7480)));


extern volatile __bit B1FILHIT1 __attribute__((address(0x7481)));


extern volatile __bit B1FILHIT2 __attribute__((address(0x7482)));


extern volatile __bit B1FILHIT3 __attribute__((address(0x7483)));


extern volatile __bit B1FILHIT4 __attribute__((address(0x7484)));


extern volatile __bit B1IE __attribute__((address(0x73F3)));


extern volatile __bit B1RB0 __attribute__((address(0x74AC)));


extern volatile __bit B1RB1 __attribute__((address(0x74AD)));


extern volatile __bit B1RTREN __attribute__((address(0x7482)));


extern volatile __bit B1RTRRO __attribute__((address(0x7485)));


extern volatile __bit B1RXFUL __attribute__((address(0x7487)));


extern volatile __bit B1RXM1 __attribute__((address(0x7486)));


extern volatile __bit B1RXRTR __attribute__((address(0x74AE)));


extern volatile __bit B1SID0 __attribute__((address(0x7495)));


extern volatile __bit B1SID1 __attribute__((address(0x7496)));


extern volatile __bit B1SID10 __attribute__((address(0x748F)));


extern volatile __bit B1SID2 __attribute__((address(0x7497)));


extern volatile __bit B1SID3 __attribute__((address(0x7488)));


extern volatile __bit B1SID4 __attribute__((address(0x7489)));


extern volatile __bit B1SID5 __attribute__((address(0x748A)));


extern volatile __bit B1SID6 __attribute__((address(0x748B)));


extern volatile __bit B1SID7 __attribute__((address(0x748C)));


extern volatile __bit B1SID8 __attribute__((address(0x748D)));


extern volatile __bit B1SID9 __attribute__((address(0x748E)));


extern volatile __bit B1SRR __attribute__((address(0x7494)));


extern volatile __bit B1TXABT __attribute__((address(0x7486)));


extern volatile __bit B1TXB3IF __attribute__((address(0x7487)));


extern volatile __bit B1TXEN __attribute__((address(0x73EB)));


extern volatile __bit B1TXERR __attribute__((address(0x7484)));


extern volatile __bit B1TXLARB __attribute__((address(0x7485)));


extern volatile __bit B1TXPRI0 __attribute__((address(0x7480)));


extern volatile __bit B1TXPRI1 __attribute__((address(0x7481)));


extern volatile __bit B1TXREQ __attribute__((address(0x7483)));


extern volatile __bit B2D00 __attribute__((address(0x7530)));


extern volatile __bit B2D01 __attribute__((address(0x7531)));


extern volatile __bit B2D02 __attribute__((address(0x7532)));


extern volatile __bit B2D03 __attribute__((address(0x7533)));


extern volatile __bit B2D04 __attribute__((address(0x7534)));


extern volatile __bit B2D05 __attribute__((address(0x7535)));


extern volatile __bit B2D06 __attribute__((address(0x7536)));


extern volatile __bit B2D07 __attribute__((address(0x7537)));


extern volatile __bit B2D10 __attribute__((address(0x7538)));


extern volatile __bit B2D11 __attribute__((address(0x7539)));


extern volatile __bit B2D12 __attribute__((address(0x753A)));


extern volatile __bit B2D13 __attribute__((address(0x753B)));


extern volatile __bit B2D14 __attribute__((address(0x753C)));


extern volatile __bit B2D15 __attribute__((address(0x753D)));


extern volatile __bit B2D16 __attribute__((address(0x753E)));


extern volatile __bit B2D17 __attribute__((address(0x753F)));


extern volatile __bit B2D20 __attribute__((address(0x7540)));


extern volatile __bit B2D21 __attribute__((address(0x7541)));


extern volatile __bit B2D22 __attribute__((address(0x7542)));


extern volatile __bit B2D23 __attribute__((address(0x7543)));


extern volatile __bit B2D24 __attribute__((address(0x7544)));


extern volatile __bit B2D25 __attribute__((address(0x7545)));


extern volatile __bit B2D26 __attribute__((address(0x7546)));


extern volatile __bit B2D27 __attribute__((address(0x7547)));


extern volatile __bit B2D30 __attribute__((address(0x7548)));


extern volatile __bit B2D31 __attribute__((address(0x7549)));


extern volatile __bit B2D32 __attribute__((address(0x754A)));


extern volatile __bit B2D33 __attribute__((address(0x754B)));


extern volatile __bit B2D34 __attribute__((address(0x754C)));


extern volatile __bit B2D35 __attribute__((address(0x754D)));


extern volatile __bit B2D36 __attribute__((address(0x754E)));


extern volatile __bit B2D37 __attribute__((address(0x754F)));


extern volatile __bit B2D40 __attribute__((address(0x7550)));


extern volatile __bit B2D41 __attribute__((address(0x7551)));


extern volatile __bit B2D42 __attribute__((address(0x7552)));


extern volatile __bit B2D43 __attribute__((address(0x7553)));


extern volatile __bit B2D44 __attribute__((address(0x7554)));


extern volatile __bit B2D45 __attribute__((address(0x7555)));


extern volatile __bit B2D46 __attribute__((address(0x7556)));


extern volatile __bit B2D47 __attribute__((address(0x7557)));


extern volatile __bit B2D50 __attribute__((address(0x7558)));


extern volatile __bit B2D51 __attribute__((address(0x7559)));


extern volatile __bit B2D52 __attribute__((address(0x755A)));


extern volatile __bit B2D53 __attribute__((address(0x755B)));


extern volatile __bit B2D54 __attribute__((address(0x755C)));


extern volatile __bit B2D55 __attribute__((address(0x755D)));


extern volatile __bit B2D56 __attribute__((address(0x755E)));


extern volatile __bit B2D57 __attribute__((address(0x755F)));


extern volatile __bit B2D60 __attribute__((address(0x7560)));


extern volatile __bit B2D61 __attribute__((address(0x7561)));


extern volatile __bit B2D62 __attribute__((address(0x7562)));


extern volatile __bit B2D63 __attribute__((address(0x7563)));


extern volatile __bit B2D64 __attribute__((address(0x7564)));


extern volatile __bit B2D65 __attribute__((address(0x7565)));


extern volatile __bit B2D66 __attribute__((address(0x7566)));


extern volatile __bit B2D67 __attribute__((address(0x7567)));


extern volatile __bit B2D70 __attribute__((address(0x7568)));


extern volatile __bit B2D71 __attribute__((address(0x7569)));


extern volatile __bit B2D72 __attribute__((address(0x756A)));


extern volatile __bit B2D73 __attribute__((address(0x756B)));


extern volatile __bit B2D74 __attribute__((address(0x756C)));


extern volatile __bit B2D75 __attribute__((address(0x756D)));


extern volatile __bit B2D76 __attribute__((address(0x756E)));


extern volatile __bit B2D77 __attribute__((address(0x756F)));


extern volatile __bit B2DLC0 __attribute__((address(0x7528)));


extern volatile __bit B2DLC1 __attribute__((address(0x7529)));


extern volatile __bit B2DLC2 __attribute__((address(0x752A)));


extern volatile __bit B2DLC3 __attribute__((address(0x752B)));


extern volatile __bit B2EID0 __attribute__((address(0x7520)));


extern volatile __bit B2EID1 __attribute__((address(0x7521)));


extern volatile __bit B2EID10 __attribute__((address(0x751A)));


extern volatile __bit B2EID11 __attribute__((address(0x751B)));


extern volatile __bit B2EID12 __attribute__((address(0x751C)));


extern volatile __bit B2EID13 __attribute__((address(0x751D)));


extern volatile __bit B2EID14 __attribute__((address(0x751E)));


extern volatile __bit B2EID15 __attribute__((address(0x751F)));


extern volatile __bit B2EID16 __attribute__((address(0x7510)));


extern volatile __bit B2EID17 __attribute__((address(0x7511)));


extern volatile __bit B2EID2 __attribute__((address(0x7522)));


extern volatile __bit B2EID3 __attribute__((address(0x7523)));


extern volatile __bit B2EID4 __attribute__((address(0x7524)));


extern volatile __bit B2EID5 __attribute__((address(0x7525)));


extern volatile __bit B2EID6 __attribute__((address(0x7526)));


extern volatile __bit B2EID7 __attribute__((address(0x7527)));


extern volatile __bit B2EID8 __attribute__((address(0x7518)));


extern volatile __bit B2EID9 __attribute__((address(0x7519)));


extern volatile __bit B2EXID __attribute__((address(0x7513)));


extern volatile __bit B2EXIDE __attribute__((address(0x7513)));


extern volatile __bit B2FILHIT0 __attribute__((address(0x7500)));


extern volatile __bit B2FILHIT1 __attribute__((address(0x7501)));


extern volatile __bit B2FILHIT2 __attribute__((address(0x7502)));


extern volatile __bit B2FILHIT3 __attribute__((address(0x7503)));


extern volatile __bit B2FILHIT4 __attribute__((address(0x7504)));


extern volatile __bit B2IE __attribute__((address(0x73F4)));


extern volatile __bit B2RB0 __attribute__((address(0x752C)));


extern volatile __bit B2RB1 __attribute__((address(0x752D)));


extern volatile __bit B2RTREN __attribute__((address(0x7502)));


extern volatile __bit B2RTRRO __attribute__((address(0x7505)));


extern volatile __bit B2RXFUL __attribute__((address(0x7507)));


extern volatile __bit B2RXM1 __attribute__((address(0x7506)));


extern volatile __bit B2RXRTR __attribute__((address(0x752E)));


extern volatile __bit B2SID0 __attribute__((address(0x7515)));


extern volatile __bit B2SID1 __attribute__((address(0x7516)));


extern volatile __bit B2SID10 __attribute__((address(0x750F)));


extern volatile __bit B2SID2 __attribute__((address(0x7517)));


extern volatile __bit B2SID3 __attribute__((address(0x7508)));


extern volatile __bit B2SID4 __attribute__((address(0x7509)));


extern volatile __bit B2SID5 __attribute__((address(0x750A)));


extern volatile __bit B2SID6 __attribute__((address(0x750B)));


extern volatile __bit B2SID7 __attribute__((address(0x750C)));


extern volatile __bit B2SID8 __attribute__((address(0x750D)));


extern volatile __bit B2SID9 __attribute__((address(0x750E)));


extern volatile __bit B2SRR __attribute__((address(0x7514)));


extern volatile __bit B2TXABT __attribute__((address(0x7506)));


extern volatile __bit B2TXB3IF __attribute__((address(0x7507)));


extern volatile __bit B2TXEN __attribute__((address(0x73EC)));


extern volatile __bit B2TXERR __attribute__((address(0x7504)));


extern volatile __bit B2TXLARB __attribute__((address(0x7505)));


extern volatile __bit B2TXPRI0 __attribute__((address(0x7500)));


extern volatile __bit B2TXPRI1 __attribute__((address(0x7501)));


extern volatile __bit B2TXREQ __attribute__((address(0x7503)));


extern volatile __bit B3D00 __attribute__((address(0x75B0)));


extern volatile __bit B3D01 __attribute__((address(0x75B1)));


extern volatile __bit B3D02 __attribute__((address(0x75B2)));


extern volatile __bit B3D03 __attribute__((address(0x75B3)));


extern volatile __bit B3D04 __attribute__((address(0x75B4)));


extern volatile __bit B3D05 __attribute__((address(0x75B5)));


extern volatile __bit B3D06 __attribute__((address(0x75B6)));


extern volatile __bit B3D07 __attribute__((address(0x75B7)));


extern volatile __bit B3D10 __attribute__((address(0x75B8)));


extern volatile __bit B3D11 __attribute__((address(0x75B9)));


extern volatile __bit B3D12 __attribute__((address(0x75BA)));


extern volatile __bit B3D13 __attribute__((address(0x75BB)));


extern volatile __bit B3D14 __attribute__((address(0x75BC)));


extern volatile __bit B3D15 __attribute__((address(0x75BD)));


extern volatile __bit B3D16 __attribute__((address(0x75BE)));


extern volatile __bit B3D17 __attribute__((address(0x75BF)));


extern volatile __bit B3D20 __attribute__((address(0x75C0)));


extern volatile __bit B3D21 __attribute__((address(0x75C1)));


extern volatile __bit B3D22 __attribute__((address(0x75C2)));


extern volatile __bit B3D23 __attribute__((address(0x75C3)));


extern volatile __bit B3D24 __attribute__((address(0x75C4)));


extern volatile __bit B3D25 __attribute__((address(0x75C5)));


extern volatile __bit B3D26 __attribute__((address(0x75C6)));


extern volatile __bit B3D27 __attribute__((address(0x75C7)));


extern volatile __bit B3D30 __attribute__((address(0x75C8)));


extern volatile __bit B3D31 __attribute__((address(0x75C9)));


extern volatile __bit B3D32 __attribute__((address(0x75CA)));


extern volatile __bit B3D33 __attribute__((address(0x75CB)));


extern volatile __bit B3D34 __attribute__((address(0x75CC)));


extern volatile __bit B3D35 __attribute__((address(0x75CD)));


extern volatile __bit B3D36 __attribute__((address(0x75CE)));


extern volatile __bit B3D37 __attribute__((address(0x75CF)));


extern volatile __bit B3D40 __attribute__((address(0x75D0)));


extern volatile __bit B3D41 __attribute__((address(0x75D1)));


extern volatile __bit B3D42 __attribute__((address(0x75D2)));


extern volatile __bit B3D43 __attribute__((address(0x75D3)));


extern volatile __bit B3D44 __attribute__((address(0x75D4)));


extern volatile __bit B3D45 __attribute__((address(0x75D5)));


extern volatile __bit B3D46 __attribute__((address(0x75D6)));


extern volatile __bit B3D47 __attribute__((address(0x75D7)));


extern volatile __bit B3D50 __attribute__((address(0x75D8)));


extern volatile __bit B3D51 __attribute__((address(0x75D9)));


extern volatile __bit B3D52 __attribute__((address(0x75DA)));


extern volatile __bit B3D53 __attribute__((address(0x75DB)));


extern volatile __bit B3D54 __attribute__((address(0x75DC)));


extern volatile __bit B3D55 __attribute__((address(0x75DD)));


extern volatile __bit B3D56 __attribute__((address(0x75DE)));


extern volatile __bit B3D57 __attribute__((address(0x75DF)));


extern volatile __bit B3D60 __attribute__((address(0x75E0)));


extern volatile __bit B3D61 __attribute__((address(0x75E1)));


extern volatile __bit B3D62 __attribute__((address(0x75E2)));


extern volatile __bit B3D63 __attribute__((address(0x75E3)));


extern volatile __bit B3D64 __attribute__((address(0x75E4)));


extern volatile __bit B3D65 __attribute__((address(0x75E5)));


extern volatile __bit B3D66 __attribute__((address(0x75E6)));


extern volatile __bit B3D67 __attribute__((address(0x75E7)));


extern volatile __bit B3D70 __attribute__((address(0x75E8)));


extern volatile __bit B3D71 __attribute__((address(0x75E9)));


extern volatile __bit B3D72 __attribute__((address(0x75EA)));


extern volatile __bit B3D73 __attribute__((address(0x75EB)));


extern volatile __bit B3D74 __attribute__((address(0x75EC)));


extern volatile __bit B3D75 __attribute__((address(0x75ED)));


extern volatile __bit B3D76 __attribute__((address(0x75EE)));


extern volatile __bit B3D77 __attribute__((address(0x75EF)));


extern volatile __bit B3DLC0 __attribute__((address(0x75A8)));


extern volatile __bit B3DLC1 __attribute__((address(0x75A9)));


extern volatile __bit B3DLC2 __attribute__((address(0x75AA)));


extern volatile __bit B3DLC3 __attribute__((address(0x75AB)));


extern volatile __bit B3EID0 __attribute__((address(0x75A0)));


extern volatile __bit B3EID1 __attribute__((address(0x75A1)));


extern volatile __bit B3EID10 __attribute__((address(0x759A)));


extern volatile __bit B3EID11 __attribute__((address(0x759B)));


extern volatile __bit B3EID12 __attribute__((address(0x759C)));


extern volatile __bit B3EID13 __attribute__((address(0x759D)));


extern volatile __bit B3EID14 __attribute__((address(0x759E)));


extern volatile __bit B3EID15 __attribute__((address(0x759F)));


extern volatile __bit B3EID16 __attribute__((address(0x7590)));


extern volatile __bit B3EID17 __attribute__((address(0x7591)));


extern volatile __bit B3EID2 __attribute__((address(0x75A2)));


extern volatile __bit B3EID3 __attribute__((address(0x75A3)));


extern volatile __bit B3EID4 __attribute__((address(0x75A4)));


extern volatile __bit B3EID5 __attribute__((address(0x75A5)));


extern volatile __bit B3EID6 __attribute__((address(0x75A6)));


extern volatile __bit B3EID7 __attribute__((address(0x75A7)));


extern volatile __bit B3EID8 __attribute__((address(0x7598)));


extern volatile __bit B3EID9 __attribute__((address(0x7599)));


extern volatile __bit B3EXID __attribute__((address(0x7593)));


extern volatile __bit B3EXIDE __attribute__((address(0x7593)));


extern volatile __bit B3FILHIT0 __attribute__((address(0x7580)));


extern volatile __bit B3FILHIT1 __attribute__((address(0x7581)));


extern volatile __bit B3FILHIT2 __attribute__((address(0x7582)));


extern volatile __bit B3FILHIT3 __attribute__((address(0x7583)));


extern volatile __bit B3FILHIT4 __attribute__((address(0x7584)));


extern volatile __bit B3IE __attribute__((address(0x73F5)));


extern volatile __bit B3RB0 __attribute__((address(0x75AC)));


extern volatile __bit B3RB1 __attribute__((address(0x75AD)));


extern volatile __bit B3RTREN __attribute__((address(0x7582)));


extern volatile __bit B3RTRRO __attribute__((address(0x7585)));


extern volatile __bit B3RXFUL __attribute__((address(0x7587)));


extern volatile __bit B3RXM1 __attribute__((address(0x7586)));


extern volatile __bit B3RXRTR __attribute__((address(0x75AE)));


extern volatile __bit B3SID0 __attribute__((address(0x7595)));


extern volatile __bit B3SID1 __attribute__((address(0x7596)));


extern volatile __bit B3SID10 __attribute__((address(0x758F)));


extern volatile __bit B3SID2 __attribute__((address(0x7597)));


extern volatile __bit B3SID3 __attribute__((address(0x7588)));


extern volatile __bit B3SID4 __attribute__((address(0x7589)));


extern volatile __bit B3SID5 __attribute__((address(0x758A)));


extern volatile __bit B3SID6 __attribute__((address(0x758B)));


extern volatile __bit B3SID7 __attribute__((address(0x758C)));


extern volatile __bit B3SID8 __attribute__((address(0x758D)));


extern volatile __bit B3SID9 __attribute__((address(0x758E)));


extern volatile __bit B3SRR __attribute__((address(0x7594)));


extern volatile __bit B3TXABT __attribute__((address(0x7586)));


extern volatile __bit B3TXB3IF __attribute__((address(0x7587)));


extern volatile __bit B3TXEN __attribute__((address(0x73ED)));


extern volatile __bit B3TXERR __attribute__((address(0x7584)));


extern volatile __bit B3TXLARB __attribute__((address(0x7585)));


extern volatile __bit B3TXPRI0 __attribute__((address(0x7580)));


extern volatile __bit B3TXPRI1 __attribute__((address(0x7581)));


extern volatile __bit B3TXREQ __attribute__((address(0x7583)));


extern volatile __bit B4D00 __attribute__((address(0x7630)));


extern volatile __bit B4D01 __attribute__((address(0x7631)));


extern volatile __bit B4D02 __attribute__((address(0x7632)));


extern volatile __bit B4D03 __attribute__((address(0x7633)));


extern volatile __bit B4D04 __attribute__((address(0x7634)));


extern volatile __bit B4D05 __attribute__((address(0x7635)));


extern volatile __bit B4D06 __attribute__((address(0x7636)));


extern volatile __bit B4D07 __attribute__((address(0x7637)));


extern volatile __bit B4D10 __attribute__((address(0x7638)));


extern volatile __bit B4D11 __attribute__((address(0x7639)));


extern volatile __bit B4D12 __attribute__((address(0x763A)));


extern volatile __bit B4D13 __attribute__((address(0x763B)));


extern volatile __bit B4D14 __attribute__((address(0x763C)));


extern volatile __bit B4D15 __attribute__((address(0x763D)));


extern volatile __bit B4D16 __attribute__((address(0x763E)));


extern volatile __bit B4D17 __attribute__((address(0x763F)));


extern volatile __bit B4D20 __attribute__((address(0x7640)));


extern volatile __bit B4D21 __attribute__((address(0x7641)));


extern volatile __bit B4D22 __attribute__((address(0x7642)));


extern volatile __bit B4D23 __attribute__((address(0x7643)));


extern volatile __bit B4D24 __attribute__((address(0x7644)));


extern volatile __bit B4D25 __attribute__((address(0x7645)));


extern volatile __bit B4D26 __attribute__((address(0x7646)));


extern volatile __bit B4D27 __attribute__((address(0x7647)));


extern volatile __bit B4D30 __attribute__((address(0x7648)));


extern volatile __bit B4D31 __attribute__((address(0x7649)));


extern volatile __bit B4D32 __attribute__((address(0x764A)));


extern volatile __bit B4D33 __attribute__((address(0x764B)));


extern volatile __bit B4D34 __attribute__((address(0x764C)));


extern volatile __bit B4D35 __attribute__((address(0x764D)));


extern volatile __bit B4D36 __attribute__((address(0x764E)));


extern volatile __bit B4D37 __attribute__((address(0x764F)));


extern volatile __bit B4D40 __attribute__((address(0x7650)));


extern volatile __bit B4D41 __attribute__((address(0x7651)));


extern volatile __bit B4D42 __attribute__((address(0x7652)));


extern volatile __bit B4D43 __attribute__((address(0x7653)));


extern volatile __bit B4D44 __attribute__((address(0x7654)));


extern volatile __bit B4D45 __attribute__((address(0x7655)));


extern volatile __bit B4D46 __attribute__((address(0x7656)));


extern volatile __bit B4D47 __attribute__((address(0x7657)));


extern volatile __bit B4D50 __attribute__((address(0x7658)));


extern volatile __bit B4D51 __attribute__((address(0x7659)));


extern volatile __bit B4D52 __attribute__((address(0x765A)));


extern volatile __bit B4D53 __attribute__((address(0x765B)));


extern volatile __bit B4D54 __attribute__((address(0x765C)));


extern volatile __bit B4D55 __attribute__((address(0x765D)));


extern volatile __bit B4D56 __attribute__((address(0x765E)));


extern volatile __bit B4D57 __attribute__((address(0x765F)));


extern volatile __bit B4D60 __attribute__((address(0x7660)));


extern volatile __bit B4D61 __attribute__((address(0x7661)));


extern volatile __bit B4D62 __attribute__((address(0x7662)));


extern volatile __bit B4D63 __attribute__((address(0x7663)));


extern volatile __bit B4D64 __attribute__((address(0x7664)));


extern volatile __bit B4D65 __attribute__((address(0x7665)));


extern volatile __bit B4D66 __attribute__((address(0x7666)));


extern volatile __bit B4D67 __attribute__((address(0x7667)));


extern volatile __bit B4D70 __attribute__((address(0x7668)));


extern volatile __bit B4D71 __attribute__((address(0x7669)));


extern volatile __bit B4D72 __attribute__((address(0x766A)));


extern volatile __bit B4D73 __attribute__((address(0x766B)));


extern volatile __bit B4D74 __attribute__((address(0x766C)));


extern volatile __bit B4D75 __attribute__((address(0x766D)));


extern volatile __bit B4D76 __attribute__((address(0x766E)));


extern volatile __bit B4D77 __attribute__((address(0x766F)));


extern volatile __bit B4DLC0 __attribute__((address(0x7628)));


extern volatile __bit B4DLC1 __attribute__((address(0x7629)));


extern volatile __bit B4DLC2 __attribute__((address(0x762A)));


extern volatile __bit B4DLC3 __attribute__((address(0x762B)));


extern volatile __bit B4EID0 __attribute__((address(0x7620)));


extern volatile __bit B4EID1 __attribute__((address(0x7621)));


extern volatile __bit B4EID10 __attribute__((address(0x761A)));


extern volatile __bit B4EID11 __attribute__((address(0x761B)));


extern volatile __bit B4EID12 __attribute__((address(0x761C)));


extern volatile __bit B4EID13 __attribute__((address(0x761D)));


extern volatile __bit B4EID14 __attribute__((address(0x761E)));


extern volatile __bit B4EID15 __attribute__((address(0x761F)));


extern volatile __bit B4EID16 __attribute__((address(0x7610)));


extern volatile __bit B4EID17 __attribute__((address(0x7611)));


extern volatile __bit B4EID2 __attribute__((address(0x7622)));


extern volatile __bit B4EID3 __attribute__((address(0x7623)));


extern volatile __bit B4EID4 __attribute__((address(0x7624)));


extern volatile __bit B4EID5 __attribute__((address(0x7625)));


extern volatile __bit B4EID6 __attribute__((address(0x7626)));


extern volatile __bit B4EID7 __attribute__((address(0x7627)));


extern volatile __bit B4EID8 __attribute__((address(0x7618)));


extern volatile __bit B4EID9 __attribute__((address(0x7619)));


extern volatile __bit B4EXID __attribute__((address(0x7613)));


extern volatile __bit B4EXIDE __attribute__((address(0x7613)));


extern volatile __bit B4FILHIT0 __attribute__((address(0x7600)));


extern volatile __bit B4FILHIT1 __attribute__((address(0x7601)));


extern volatile __bit B4FILHIT2 __attribute__((address(0x7602)));


extern volatile __bit B4FILHIT3 __attribute__((address(0x7603)));


extern volatile __bit B4FILHIT4 __attribute__((address(0x7604)));


extern volatile __bit B4IE __attribute__((address(0x73F6)));


extern volatile __bit B4RB0 __attribute__((address(0x762C)));


extern volatile __bit B4RB1 __attribute__((address(0x762D)));


extern volatile __bit B4RTREN __attribute__((address(0x7602)));


extern volatile __bit B4RTRRO __attribute__((address(0x7605)));


extern volatile __bit B4RXFUL __attribute__((address(0x7607)));


extern volatile __bit B4RXM1 __attribute__((address(0x7606)));


extern volatile __bit B4RXRTR __attribute__((address(0x762E)));


extern volatile __bit B4SID0 __attribute__((address(0x7615)));


extern volatile __bit B4SID1 __attribute__((address(0x7616)));


extern volatile __bit B4SID10 __attribute__((address(0x760F)));


extern volatile __bit B4SID2 __attribute__((address(0x7617)));


extern volatile __bit B4SID3 __attribute__((address(0x7608)));


extern volatile __bit B4SID4 __attribute__((address(0x7609)));


extern volatile __bit B4SID5 __attribute__((address(0x760A)));


extern volatile __bit B4SID6 __attribute__((address(0x760B)));


extern volatile __bit B4SID7 __attribute__((address(0x760C)));


extern volatile __bit B4SID8 __attribute__((address(0x760D)));


extern volatile __bit B4SID9 __attribute__((address(0x760E)));


extern volatile __bit B4SRR __attribute__((address(0x7614)));


extern volatile __bit B4TXABT __attribute__((address(0x7606)));


extern volatile __bit B4TXB3IF __attribute__((address(0x7607)));


extern volatile __bit B4TXEN __attribute__((address(0x73EE)));


extern volatile __bit B4TXERR __attribute__((address(0x7604)));


extern volatile __bit B4TXLARB __attribute__((address(0x7605)));


extern volatile __bit B4TXPRI0 __attribute__((address(0x7600)));


extern volatile __bit B4TXPRI1 __attribute__((address(0x7601)));


extern volatile __bit B4TXREQ __attribute__((address(0x7603)));


extern volatile __bit B5D00 __attribute__((address(0x76B0)));


extern volatile __bit B5D01 __attribute__((address(0x76B1)));


extern volatile __bit B5D02 __attribute__((address(0x76B2)));


extern volatile __bit B5D03 __attribute__((address(0x76B3)));


extern volatile __bit B5D04 __attribute__((address(0x76B4)));


extern volatile __bit B5D05 __attribute__((address(0x76B5)));


extern volatile __bit B5D06 __attribute__((address(0x76B6)));


extern volatile __bit B5D07 __attribute__((address(0x76B7)));


extern volatile __bit B5D10 __attribute__((address(0x76B8)));


extern volatile __bit B5D11 __attribute__((address(0x76B9)));


extern volatile __bit B5D12 __attribute__((address(0x76BA)));


extern volatile __bit B5D13 __attribute__((address(0x76BB)));


extern volatile __bit B5D14 __attribute__((address(0x76BC)));


extern volatile __bit B5D15 __attribute__((address(0x76BD)));


extern volatile __bit B5D16 __attribute__((address(0x76BE)));


extern volatile __bit B5D17 __attribute__((address(0x76BF)));


extern volatile __bit B5D20 __attribute__((address(0x76C0)));


extern volatile __bit B5D21 __attribute__((address(0x76C1)));


extern volatile __bit B5D22 __attribute__((address(0x76C2)));


extern volatile __bit B5D23 __attribute__((address(0x76C3)));


extern volatile __bit B5D24 __attribute__((address(0x76C4)));


extern volatile __bit B5D25 __attribute__((address(0x76C5)));


extern volatile __bit B5D26 __attribute__((address(0x76C6)));


extern volatile __bit B5D27 __attribute__((address(0x76C7)));


extern volatile __bit B5D30 __attribute__((address(0x76C8)));


extern volatile __bit B5D31 __attribute__((address(0x76C9)));


extern volatile __bit B5D32 __attribute__((address(0x76CA)));


extern volatile __bit B5D33 __attribute__((address(0x76CB)));


extern volatile __bit B5D34 __attribute__((address(0x76CC)));


extern volatile __bit B5D35 __attribute__((address(0x76CD)));


extern volatile __bit B5D36 __attribute__((address(0x76CE)));


extern volatile __bit B5D37 __attribute__((address(0x76CF)));


extern volatile __bit B5D40 __attribute__((address(0x76D0)));


extern volatile __bit B5D41 __attribute__((address(0x76D1)));


extern volatile __bit B5D42 __attribute__((address(0x76D2)));


extern volatile __bit B5D43 __attribute__((address(0x76D3)));


extern volatile __bit B5D44 __attribute__((address(0x76D4)));


extern volatile __bit B5D45 __attribute__((address(0x76D5)));


extern volatile __bit B5D46 __attribute__((address(0x76D6)));


extern volatile __bit B5D47 __attribute__((address(0x76D7)));


extern volatile __bit B5D50 __attribute__((address(0x76D8)));


extern volatile __bit B5D51 __attribute__((address(0x76D9)));


extern volatile __bit B5D52 __attribute__((address(0x76DA)));


extern volatile __bit B5D53 __attribute__((address(0x76DB)));


extern volatile __bit B5D54 __attribute__((address(0x76DC)));


extern volatile __bit B5D55 __attribute__((address(0x76DD)));


extern volatile __bit B5D56 __attribute__((address(0x76DE)));


extern volatile __bit B5D57 __attribute__((address(0x76DF)));


extern volatile __bit B5D60 __attribute__((address(0x76E0)));


extern volatile __bit B5D61 __attribute__((address(0x76E1)));


extern volatile __bit B5D62 __attribute__((address(0x76E2)));


extern volatile __bit B5D63 __attribute__((address(0x76E3)));


extern volatile __bit B5D64 __attribute__((address(0x76E4)));


extern volatile __bit B5D65 __attribute__((address(0x76E5)));


extern volatile __bit B5D66 __attribute__((address(0x76E6)));


extern volatile __bit B5D67 __attribute__((address(0x76E7)));


extern volatile __bit B5D70 __attribute__((address(0x76E8)));


extern volatile __bit B5D71 __attribute__((address(0x76E9)));


extern volatile __bit B5D72 __attribute__((address(0x76EA)));


extern volatile __bit B5D73 __attribute__((address(0x76EB)));


extern volatile __bit B5D74 __attribute__((address(0x76EC)));


extern volatile __bit B5D75 __attribute__((address(0x76ED)));


extern volatile __bit B5D76 __attribute__((address(0x76EE)));


extern volatile __bit B5D77 __attribute__((address(0x76EF)));


extern volatile __bit B5DLC0 __attribute__((address(0x76A8)));


extern volatile __bit B5DLC1 __attribute__((address(0x76A9)));


extern volatile __bit B5DLC2 __attribute__((address(0x76AA)));


extern volatile __bit B5DLC3 __attribute__((address(0x76AB)));


extern volatile __bit B5EID0 __attribute__((address(0x76A0)));


extern volatile __bit B5EID1 __attribute__((address(0x76A1)));


extern volatile __bit B5EID10 __attribute__((address(0x769A)));


extern volatile __bit B5EID11 __attribute__((address(0x769B)));


extern volatile __bit B5EID12 __attribute__((address(0x769C)));


extern volatile __bit B5EID13 __attribute__((address(0x769D)));


extern volatile __bit B5EID14 __attribute__((address(0x769E)));


extern volatile __bit B5EID15 __attribute__((address(0x769F)));


extern volatile __bit B5EID16 __attribute__((address(0x7690)));


extern volatile __bit B5EID17 __attribute__((address(0x7691)));


extern volatile __bit B5EID2 __attribute__((address(0x76A2)));


extern volatile __bit B5EID3 __attribute__((address(0x76A3)));


extern volatile __bit B5EID4 __attribute__((address(0x76A4)));


extern volatile __bit B5EID5 __attribute__((address(0x76A5)));


extern volatile __bit B5EID6 __attribute__((address(0x76A6)));


extern volatile __bit B5EID7 __attribute__((address(0x76A7)));


extern volatile __bit B5EID8 __attribute__((address(0x7698)));


extern volatile __bit B5EID9 __attribute__((address(0x7699)));


extern volatile __bit B5EXID __attribute__((address(0x7693)));


extern volatile __bit B5EXIDE __attribute__((address(0x7693)));


extern volatile __bit B5FILHIT0 __attribute__((address(0x7680)));


extern volatile __bit B5FILHIT1 __attribute__((address(0x7681)));


extern volatile __bit B5FILHIT2 __attribute__((address(0x7682)));


extern volatile __bit B5FILHIT3 __attribute__((address(0x7683)));


extern volatile __bit B5FILHIT4 __attribute__((address(0x7684)));


extern volatile __bit B5IE __attribute__((address(0x73F7)));


extern volatile __bit B5RB0 __attribute__((address(0x76AC)));


extern volatile __bit B5RB1 __attribute__((address(0x76AD)));


extern volatile __bit B5RTREN __attribute__((address(0x7682)));


extern volatile __bit B5RTRRO __attribute__((address(0x7685)));


extern volatile __bit B5RXFUL __attribute__((address(0x7687)));


extern volatile __bit B5RXM1 __attribute__((address(0x7686)));


extern volatile __bit B5RXRTR __attribute__((address(0x76AE)));


extern volatile __bit B5SID0 __attribute__((address(0x7695)));


extern volatile __bit B5SID1 __attribute__((address(0x7696)));


extern volatile __bit B5SID10 __attribute__((address(0x768F)));


extern volatile __bit B5SID2 __attribute__((address(0x7697)));


extern volatile __bit B5SID3 __attribute__((address(0x7688)));


extern volatile __bit B5SID4 __attribute__((address(0x7689)));


extern volatile __bit B5SID5 __attribute__((address(0x768A)));


extern volatile __bit B5SID6 __attribute__((address(0x768B)));


extern volatile __bit B5SID7 __attribute__((address(0x768C)));


extern volatile __bit B5SID8 __attribute__((address(0x768D)));


extern volatile __bit B5SID9 __attribute__((address(0x768E)));


extern volatile __bit B5SRR __attribute__((address(0x7694)));


extern volatile __bit B5TXABT __attribute__((address(0x7686)));


extern volatile __bit B5TXBIF __attribute__((address(0x7687)));


extern volatile __bit B5TXEN __attribute__((address(0x73EF)));


extern volatile __bit B5TXERR __attribute__((address(0x7684)));


extern volatile __bit B5TXLARB __attribute__((address(0x7685)));


extern volatile __bit B5TXPRI0 __attribute__((address(0x7680)));


extern volatile __bit B5TXPRI1 __attribute__((address(0x7681)));


extern volatile __bit B5TXREQ __attribute__((address(0x7683)));


extern volatile __bit BCLIE __attribute__((address(0x7D03)));


extern volatile __bit BCLIF __attribute__((address(0x7D0B)));


extern volatile __bit BCLIP __attribute__((address(0x7D13)));


extern volatile __bit BF __attribute__((address(0x7E38)));


extern volatile __bit BGVST __attribute__((address(0x7D46)));


extern volatile __bit BOR __attribute__((address(0x7E80)));


extern volatile __bit BRG161 __attribute__((address(0x7D3B)));


extern volatile __bit BRG162 __attribute__((address(0x7DCB)));


extern volatile __bit BRGH1 __attribute__((address(0x7D62)));


extern volatile __bit BRGH2 __attribute__((address(0x7DD2)));


extern volatile __bit BRP0 __attribute__((address(0x7218)));


extern volatile __bit BRP1 __attribute__((address(0x7219)));


extern volatile __bit BRP2 __attribute__((address(0x721A)));


extern volatile __bit BRP3 __attribute__((address(0x721B)));


extern volatile __bit BRP4 __attribute__((address(0x721C)));


extern volatile __bit BRP5 __attribute__((address(0x721D)));


extern volatile __bit C1CH0 __attribute__((address(0x7AF8)));


extern volatile __bit C1CH1 __attribute__((address(0x7AF9)));


extern volatile __bit C1TSEL __attribute__((address(0x7CC8)));


extern volatile __bit C2OUT __attribute__((address(0x7DA6)));


extern volatile __bit C2TSEL __attribute__((address(0x7CC9)));


extern volatile __bit C3OUT __attribute__((address(0x7DA7)));


extern volatile __bit C3TSEL __attribute__((address(0x7CCA)));


extern volatile __bit C4TSEL __attribute__((address(0x7CCB)));


extern volatile __bit C5TSEL __attribute__((address(0x7CCC)));


extern volatile __bit CANCAP __attribute__((address(0x7B84)));


extern volatile __bit CARRY __attribute__((address(0x7EC0)));


extern volatile __bit CCH01 __attribute__((address(0x7AF8)));


extern volatile __bit CCH02 __attribute__((address(0x7AF0)));


extern volatile __bit CCH05 __attribute__((address(0x7BC0)));


extern volatile __bit CCH11 __attribute__((address(0x7AF9)));


extern volatile __bit CCH12 __attribute__((address(0x7AF1)));


extern volatile __bit CCH15 __attribute__((address(0x7BC1)));


extern volatile __bit CCIP3IP __attribute__((address(0x7DC0)));


extern volatile __bit CCP1IE __attribute__((address(0x7D19)));


extern volatile __bit CCP1IF __attribute__((address(0x7D21)));


extern volatile __bit CCP1IP __attribute__((address(0x7D29)));


extern volatile __bit CCP1M0 __attribute__((address(0x7DD8)));


extern volatile __bit CCP1M1 __attribute__((address(0x7DD9)));


extern volatile __bit CCP1M2 __attribute__((address(0x7DDA)));


extern volatile __bit CCP1M3 __attribute__((address(0x7DDB)));


extern volatile __bit CCP1MD __attribute__((address(0x7ACB)));


extern volatile __bit CCP1OD __attribute__((address(0x7C8A)));


extern volatile __bit CCP1X __attribute__((address(0x7DDD)));


extern volatile __bit CCP1Y __attribute__((address(0x7DDC)));


extern volatile __bit CCP2 __attribute__((address(0x7C11)));


extern volatile __bit CCP2IE __attribute__((address(0x7D1A)));


extern volatile __bit CCP2IF __attribute__((address(0x7D22)));


extern volatile __bit CCP2IP __attribute__((address(0x7D2A)));


extern volatile __bit CCP2M0 __attribute__((address(0x7A80)));


extern volatile __bit CCP2M1 __attribute__((address(0x7A81)));


extern volatile __bit CCP2M2 __attribute__((address(0x7A82)));


extern volatile __bit CCP2M3 __attribute__((address(0x7A83)));


extern volatile __bit CCP2MD __attribute__((address(0x7ACC)));


extern volatile __bit CCP2OD __attribute__((address(0x7C8B)));


extern volatile __bit CCP2X __attribute__((address(0x7A85)));


extern volatile __bit CCP2Y __attribute__((address(0x7A84)));


extern volatile __bit CCP2_PA2 __attribute__((address(0x7C0B)));


extern volatile __bit CCP3IE __attribute__((address(0x7DB0)));


extern volatile __bit CCP3IF __attribute__((address(0x7DB8)));


extern volatile __bit CCP3IP __attribute__((address(0x7DC0)));


extern volatile __bit CCP3M0 __attribute__((address(0x7A68)));


extern volatile __bit CCP3M1 __attribute__((address(0x7A69)));


extern volatile __bit CCP3M2 __attribute__((address(0x7A6A)));


extern volatile __bit CCP3M3 __attribute__((address(0x7A6B)));


extern volatile __bit CCP3MD __attribute__((address(0x7ACD)));


extern volatile __bit CCP3OD __attribute__((address(0x7C8C)));


extern volatile __bit CCP3X __attribute__((address(0x7A6D)));


extern volatile __bit CCP3Y __attribute__((address(0x7A6C)));


extern volatile __bit CCP4IE __attribute__((address(0x7DB1)));


extern volatile __bit CCP4IF __attribute__((address(0x7DB9)));


extern volatile __bit CCP4IP __attribute__((address(0x7DC1)));


extern volatile __bit CCP4M0 __attribute__((address(0x7A50)));


extern volatile __bit CCP4M1 __attribute__((address(0x7A51)));


extern volatile __bit CCP4M2 __attribute__((address(0x7A52)));


extern volatile __bit CCP4M3 __attribute__((address(0x7A53)));


extern volatile __bit CCP4MD __attribute__((address(0x7ACE)));


extern volatile __bit CCP4OD __attribute__((address(0x7C8D)));


extern volatile __bit CCP4X __attribute__((address(0x7A55)));


extern volatile __bit CCP4Y __attribute__((address(0x7A54)));


extern volatile __bit CCP5IE __attribute__((address(0x7DB2)));


extern volatile __bit CCP5IF __attribute__((address(0x7DBA)));


extern volatile __bit CCP5IP __attribute__((address(0x7DC2)));


extern volatile __bit CCP5M0 __attribute__((address(0x7A38)));


extern volatile __bit CCP5M1 __attribute__((address(0x7A39)));


extern volatile __bit CCP5M2 __attribute__((address(0x7A3A)));


extern volatile __bit CCP5M3 __attribute__((address(0x7A3B)));


extern volatile __bit CCP5MD __attribute__((address(0x7ACF)));


extern volatile __bit CCP5OD __attribute__((address(0x7C8E)));


extern volatile __bit CCP5X __attribute__((address(0x7A3D)));


extern volatile __bit CCP5Y __attribute__((address(0x7A3C)));


extern volatile __bit CCP9E __attribute__((address(0x7C23)));


extern volatile __bit CFGS __attribute__((address(0x7BFE)));


extern volatile __bit CHS0 __attribute__((address(0x7E12)));


extern volatile __bit CHS1 __attribute__((address(0x7E13)));


extern volatile __bit CHS2 __attribute__((address(0x7E14)));


extern volatile __bit CHS3 __attribute__((address(0x7E15)));


extern volatile __bit CHS4 __attribute__((address(0x7E16)));


extern volatile __bit CHSN0 __attribute__((address(0x7E08)));


extern volatile __bit CHSN1 __attribute__((address(0x7E09)));


extern volatile __bit CHSN2 __attribute__((address(0x7E0A)));


extern volatile __bit CHSN3 __attribute__((address(0x7E0B)));


extern volatile __bit CKE __attribute__((address(0x7E3E)));


extern volatile __bit CKP __attribute__((address(0x7E34)));


extern volatile __bit CKTXP __attribute__((address(0x7D3C)));


extern volatile __bit CLKSEL __attribute__((address(0x7B80)));


extern volatile __bit CM __attribute__((address(0x7E85)));


extern volatile __bit CMP1IE __attribute__((address(0x7DB4)));


extern volatile __bit CMP1IF __attribute__((address(0x7DBC)));


extern volatile __bit CMP1IP __attribute__((address(0x7DC4)));


extern volatile __bit CMP1MD __attribute__((address(0x7AB8)));


extern volatile __bit CMP1OUT __attribute__((address(0x7DA6)));


extern volatile __bit CMP2IE __attribute__((address(0x7DB5)));


extern volatile __bit CMP2IF __attribute__((address(0x7DBD)));


extern volatile __bit CMP2IP __attribute__((address(0x7DC5)));


extern volatile __bit CMP2MD __attribute__((address(0x7AB9)));


extern volatile __bit CMP2OUT __attribute__((address(0x7DA7)));


extern volatile __bit CMPL0 __attribute__((address(0x7CE6)));


extern volatile __bit CMPL1 __attribute__((address(0x7CE7)));


extern volatile __bit COE1 __attribute__((address(0x7AFE)));


extern volatile __bit COE2 __attribute__((address(0x7AF6)));


extern volatile __bit CON1 __attribute__((address(0x7AFF)));


extern volatile __bit CON2 __attribute__((address(0x7AF7)));


extern volatile __bit CPOL1 __attribute__((address(0x7AFD)));


extern volatile __bit CPOL2 __attribute__((address(0x7AF5)));


extern volatile __bit CREF1 __attribute__((address(0x7AFA)));


extern volatile __bit CREF2 __attribute__((address(0x7AF2)));


extern volatile __bit CREN1 __attribute__((address(0x7D5C)));


extern volatile __bit CREN2 __attribute__((address(0x7D34)));


extern volatile __bit CSRC1 __attribute__((address(0x7D67)));


extern volatile __bit CSRC2 __attribute__((address(0x7DD7)));


extern volatile __bit CTMUDS __attribute__((address(0x7AB0)));


extern volatile __bit CTMUEN __attribute__((address(0x7AAF)));


extern volatile __bit CTMUIE __attribute__((address(0x7D1B)));


extern volatile __bit CTMUIF __attribute__((address(0x7D23)));


extern volatile __bit CTMUIP __attribute__((address(0x7D2B)));


extern volatile __bit CTMUMD __attribute__((address(0x7AC6)));


extern volatile __bit CTMUSIDL __attribute__((address(0x7AAD)));


extern volatile __bit CTTRIG __attribute__((address(0x7AA8)));


extern volatile __bit CVR0 __attribute__((address(0x7DA8)));


extern volatile __bit CVR1 __attribute__((address(0x7DA9)));


extern volatile __bit CVR2 __attribute__((address(0x7DAA)));


extern volatile __bit CVR3 __attribute__((address(0x7DAB)));


extern volatile __bit CVR4 __attribute__((address(0x7DAC)));


extern volatile __bit CVREN __attribute__((address(0x7DAF)));


extern volatile __bit CVROE __attribute__((address(0x7DAE)));


extern volatile __bit CVROEN __attribute__((address(0x7DAE)));


extern volatile __bit CVRR __attribute__((address(0x7DAD)));


extern volatile __bit CVRSS __attribute__((address(0x7DAD)));


extern volatile __bit DA __attribute__((address(0x7E3D)));


extern volatile __bit DATA_ADDRESS __attribute__((address(0x7E3D)));


extern volatile __bit DC __attribute__((address(0x7EC1)));


extern volatile __bit DC1B0 __attribute__((address(0x7DDC)));


extern volatile __bit DC1B1 __attribute__((address(0x7DDD)));


extern volatile __bit DC2B0 __attribute__((address(0x7A84)));


extern volatile __bit DC2B1 __attribute__((address(0x7A85)));


extern volatile __bit DC3B0 __attribute__((address(0x7A6C)));


extern volatile __bit DC3B1 __attribute__((address(0x7A6D)));


extern volatile __bit DC4B0 __attribute__((address(0x7A54)));


extern volatile __bit DC4B1 __attribute__((address(0x7A55)));


extern volatile __bit DC5B0 __attribute__((address(0x7A3C)));


extern volatile __bit DC5B1 __attribute__((address(0x7A3D)));


extern volatile __bit DONE __attribute__((address(0x7E11)));


extern volatile __bit DRXB0IE __attribute__((address(0x73F0)));


extern volatile __bit DRXB1IE __attribute__((address(0x73F1)));


extern volatile __bit DTRXP __attribute__((address(0x7D3D)));


extern volatile __bit DTRXP1 __attribute__((address(0x7D3D)));


extern volatile __bit DTRXP2 __attribute__((address(0x7DCD)));


extern volatile __bit D_A __attribute__((address(0x7E3D)));


extern volatile __bit D_NOT_A __attribute__((address(0x7E3D)));


extern volatile __bit D_nA __attribute__((address(0x7E3D)));


extern volatile __bit EBDIS __attribute__((address(0x7E5F)));


extern volatile __bit ECANMD __attribute__((address(0x7ABA)));


extern volatile __bit ECCP1AS0 __attribute__((address(0x7DFC)));


extern volatile __bit ECCP1AS1 __attribute__((address(0x7DFD)));


extern volatile __bit ECCP1AS2 __attribute__((address(0x7DFE)));


extern volatile __bit ECCP1ASE __attribute__((address(0x7DFF)));


extern volatile __bit EDG1POL __attribute__((address(0x7AA4)));


extern volatile __bit EDG1SEL0 __attribute__((address(0x7AA2)));


extern volatile __bit EDG1SEL1 __attribute__((address(0x7AA3)));


extern volatile __bit EDG1STAT __attribute__((address(0x7AA0)));


extern volatile __bit EDG2POL __attribute__((address(0x7AA7)));


extern volatile __bit EDG2SEL0 __attribute__((address(0x7AA5)));


extern volatile __bit EDG2SEL1 __attribute__((address(0x7AA6)));


extern volatile __bit EDG2STAT __attribute__((address(0x7AA1)));


extern volatile __bit EDGEN __attribute__((address(0x7AAB)));


extern volatile __bit EDGSEQEN __attribute__((address(0x7AAA)));


extern volatile __bit EDI16 __attribute__((address(0x7990)));


extern volatile __bit EDI17 __attribute__((address(0x7991)));


extern volatile __bit EEFS __attribute__((address(0x7BFE)));


extern volatile __bit EEIE __attribute__((address(0x7DB6)));


extern volatile __bit EEIF __attribute__((address(0x7DBE)));


extern volatile __bit EEIP __attribute__((address(0x7DC6)));


extern volatile __bit EEPGD __attribute__((address(0x7BFF)));


extern volatile __bit EMBMD __attribute__((address(0x7AC0)));


extern volatile __bit ENDRHI __attribute__((address(0x7B85)));


extern volatile __bit ERRIE __attribute__((address(0x7BB5)));


extern volatile __bit ERRIF __attribute__((address(0x7BBD)));


extern volatile __bit ERRIP __attribute__((address(0x7BC5)));


extern volatile __bit EVPOL01 __attribute__((address(0x7AFB)));


extern volatile __bit EVPOL02 __attribute__((address(0x7AF3)));


extern volatile __bit EVPOL05 __attribute__((address(0x7BC3)));


extern volatile __bit EVPOL11 __attribute__((address(0x7AFC)));


extern volatile __bit EVPOL12 __attribute__((address(0x7AF4)));


extern volatile __bit EVPOL15 __attribute__((address(0x7BC4)));


extern volatile __bit EWARN __attribute__((address(0x7B88)));


extern volatile __bit EWIN0 __attribute__((address(0x7B90)));


extern volatile __bit EWIN1 __attribute__((address(0x7B91)));


extern volatile __bit EWIN2 __attribute__((address(0x7B92)));


extern volatile __bit EWIN3 __attribute__((address(0x7B93)));


extern volatile __bit EWIN4 __attribute__((address(0x7B94)));


extern volatile __bit F0BP_0 __attribute__((address(0x7388)));


extern volatile __bit F0BP_01 __attribute__((address(0x7388)));


extern volatile __bit F0BP_1 __attribute__((address(0x7389)));


extern volatile __bit F0BP_2 __attribute__((address(0x738A)));


extern volatile __bit F0BP_3 __attribute__((address(0x738B)));


extern volatile __bit F10BP_0 __attribute__((address(0x73B0)));


extern volatile __bit F10BP_01 __attribute__((address(0x73B0)));


extern volatile __bit F10BP_1 __attribute__((address(0x73B1)));


extern volatile __bit F10BP_2 __attribute__((address(0x73B2)));


extern volatile __bit F10BP_3 __attribute__((address(0x73B3)));


extern volatile __bit F11BP_0 __attribute__((address(0x73B4)));


extern volatile __bit F11BP_1 __attribute__((address(0x73B5)));


extern volatile __bit F11BP_2 __attribute__((address(0x73B6)));


extern volatile __bit F11BP_3 __attribute__((address(0x73B7)));


extern volatile __bit F12BP_0 __attribute__((address(0x73B8)));


extern volatile __bit F12BP_01 __attribute__((address(0x73B8)));


extern volatile __bit F12BP_1 __attribute__((address(0x73B9)));


extern volatile __bit F12BP_2 __attribute__((address(0x73BA)));


extern volatile __bit F12BP_3 __attribute__((address(0x73BB)));


extern volatile __bit F13BP_0 __attribute__((address(0x73BC)));


extern volatile __bit F13BP_1 __attribute__((address(0x73BD)));


extern volatile __bit F13BP_2 __attribute__((address(0x73BE)));


extern volatile __bit F13BP_3 __attribute__((address(0x73BF)));


extern volatile __bit F14BP_0 __attribute__((address(0x73C0)));


extern volatile __bit F14BP_01 __attribute__((address(0x73C0)));


extern volatile __bit F14BP_1 __attribute__((address(0x73C1)));


extern volatile __bit F14BP_2 __attribute__((address(0x73C2)));


extern volatile __bit F14BP_3 __attribute__((address(0x73C3)));


extern volatile __bit F15BP_0 __attribute__((address(0x73C4)));


extern volatile __bit F15BP_1 __attribute__((address(0x73C5)));


extern volatile __bit F15BP_2 __attribute__((address(0x73C6)));


extern volatile __bit F15BP_3 __attribute__((address(0x73C7)));


extern volatile __bit F1BP_0 __attribute__((address(0x738C)));


extern volatile __bit F1BP_1 __attribute__((address(0x738D)));


extern volatile __bit F1BP_2 __attribute__((address(0x738E)));


extern volatile __bit F1BP_3 __attribute__((address(0x738F)));


extern volatile __bit F2BP_0 __attribute__((address(0x7390)));


extern volatile __bit F2BP_01 __attribute__((address(0x7390)));


extern volatile __bit F2BP_1 __attribute__((address(0x7391)));


extern volatile __bit F2BP_2 __attribute__((address(0x7392)));


extern volatile __bit F2BP_3 __attribute__((address(0x7393)));


extern volatile __bit F3BP_0 __attribute__((address(0x7394)));


extern volatile __bit F3BP_1 __attribute__((address(0x7395)));


extern volatile __bit F3BP_2 __attribute__((address(0x7396)));


extern volatile __bit F3BP_3 __attribute__((address(0x7397)));


extern volatile __bit F4BP_0 __attribute__((address(0x7398)));


extern volatile __bit F4BP_01 __attribute__((address(0x7398)));


extern volatile __bit F4BP_1 __attribute__((address(0x7399)));


extern volatile __bit F4BP_2 __attribute__((address(0x739A)));


extern volatile __bit F4BP_3 __attribute__((address(0x739B)));


extern volatile __bit F5BP_0 __attribute__((address(0x739C)));


extern volatile __bit F5BP_1 __attribute__((address(0x739D)));


extern volatile __bit F5BP_2 __attribute__((address(0x739E)));


extern volatile __bit F5BP_3 __attribute__((address(0x739F)));


extern volatile __bit F6BP_0 __attribute__((address(0x73A0)));


extern volatile __bit F6BP_01 __attribute__((address(0x73A0)));


extern volatile __bit F6BP_1 __attribute__((address(0x73A1)));


extern volatile __bit F6BP_2 __attribute__((address(0x73A2)));


extern volatile __bit F6BP_3 __attribute__((address(0x73A3)));


extern volatile __bit F7BP_0 __attribute__((address(0x73A4)));


extern volatile __bit F7BP_1 __attribute__((address(0x73A5)));


extern volatile __bit F7BP_2 __attribute__((address(0x73A6)));


extern volatile __bit F7BP_3 __attribute__((address(0x73A7)));


extern volatile __bit F8BP_0 __attribute__((address(0x73A8)));


extern volatile __bit F8BP_01 __attribute__((address(0x73A8)));


extern volatile __bit F8BP_1 __attribute__((address(0x73A9)));


extern volatile __bit F8BP_2 __attribute__((address(0x73AA)));


extern volatile __bit F8BP_3 __attribute__((address(0x73AB)));


extern volatile __bit F9BP_0 __attribute__((address(0x73AC)));


extern volatile __bit F9BP_1 __attribute__((address(0x73AD)));


extern volatile __bit F9BP_2 __attribute__((address(0x73AE)));


extern volatile __bit F9BP_3 __attribute__((address(0x73AF)));


extern volatile __bit FERR1 __attribute__((address(0x7D5A)));


extern volatile __bit FERR2 __attribute__((address(0x7D32)));


extern volatile __bit FIFOEMPTY __attribute__((address(0x7B8F)));


extern volatile __bit FIFOWM __attribute__((address(0x7B95)));


extern volatile __bit FIFOWMIE __attribute__((address(0x7BB0)));


extern volatile __bit FIFOWMIF __attribute__((address(0x7BB8)));


extern volatile __bit FIFOWMIP __attribute__((address(0x7BC0)));


extern volatile __bit FIL0_0 __attribute__((address(0x73C8)));


extern volatile __bit FIL0_1 __attribute__((address(0x73C9)));


extern volatile __bit FIL10_0 __attribute__((address(0x73DC)));


extern volatile __bit FIL10_1 __attribute__((address(0x73DD)));


extern volatile __bit FIL11_0 __attribute__((address(0x73DE)));


extern volatile __bit FIL11_1 __attribute__((address(0x73DF)));


extern volatile __bit FIL12_0 __attribute__((address(0x73E0)));


extern volatile __bit FIL12_1 __attribute__((address(0x73E1)));


extern volatile __bit FIL13_0 __attribute__((address(0x73E2)));


extern volatile __bit FIL13_1 __attribute__((address(0x73E3)));


extern volatile __bit FIL14_0 __attribute__((address(0x73E4)));


extern volatile __bit FIL14_1 __attribute__((address(0x73E5)));


extern volatile __bit FIL15_0 __attribute__((address(0x73E6)));


extern volatile __bit FIL15_1 __attribute__((address(0x73E7)));


extern volatile __bit FIL1_0 __attribute__((address(0x73CA)));


extern volatile __bit FIL1_1 __attribute__((address(0x73CB)));


extern volatile __bit FIL2_0 __attribute__((address(0x73CC)));


extern volatile __bit FIL2_1 __attribute__((address(0x73CD)));


extern volatile __bit FIL3_0 __attribute__((address(0x73CE)));


extern volatile __bit FIL3_1 __attribute__((address(0x73CF)));


extern volatile __bit FIL4_0 __attribute__((address(0x73D0)));


extern volatile __bit FIL4_1 __attribute__((address(0x73D1)));


extern volatile __bit FIL5_0 __attribute__((address(0x73D2)));


extern volatile __bit FIL5_1 __attribute__((address(0x73D3)));


extern volatile __bit FIL6_0 __attribute__((address(0x73D4)));


extern volatile __bit FIL6_1 __attribute__((address(0x73D5)));


extern volatile __bit FIL7_0 __attribute__((address(0x73D6)));


extern volatile __bit FIL7_1 __attribute__((address(0x73D7)));


extern volatile __bit FIL8_0 __attribute__((address(0x73D8)));


extern volatile __bit FIL8_1 __attribute__((address(0x73D9)));


extern volatile __bit FIL9_0 __attribute__((address(0x73DA)));


extern volatile __bit FIL9_1 __attribute__((address(0x73DB)));


extern volatile __bit FLC0 __attribute__((address(0x7380)));


extern volatile __bit FLC1 __attribute__((address(0x7381)));


extern volatile __bit FLC2 __attribute__((address(0x7382)));


extern volatile __bit FLC3 __attribute__((address(0x7383)));


extern volatile __bit FLC4 __attribute__((address(0x7384)));


extern volatile __bit FREE __attribute__((address(0x7BFC)));


extern volatile __bit GCEN __attribute__((address(0x7E2F)));


extern volatile __bit GIE __attribute__((address(0x7F97)));


extern volatile __bit GIEH __attribute__((address(0x7F97)));


extern volatile __bit GIEL __attribute__((address(0x7F96)));


extern volatile __bit GIE_GIEH __attribute__((address(0x7F97)));


extern volatile __bit GO __attribute__((address(0x7E11)));


extern volatile __bit GODONE __attribute__((address(0x7E11)));


extern volatile __bit GO_DONE __attribute__((address(0x7E11)));


extern volatile __bit GO_NOT_DONE __attribute__((address(0x7E11)));


extern volatile __bit GO_nDONE __attribute__((address(0x7E11)));


extern volatile __bit HFIOFS __attribute__((address(0x7E9A)));


extern volatile __bit HLVDEN __attribute__((address(0x7D44)));


extern volatile __bit HLVDIE __attribute__((address(0x7D02)));


extern volatile __bit HLVDIF __attribute__((address(0x7D0A)));


extern volatile __bit HLVDIP __attribute__((address(0x7D12)));


extern volatile __bit HLVDL0 __attribute__((address(0x7D40)));


extern volatile __bit HLVDL1 __attribute__((address(0x7D41)));


extern volatile __bit HLVDL2 __attribute__((address(0x7D42)));


extern volatile __bit HLVDL3 __attribute__((address(0x7D43)));


extern volatile __bit I2C_DAT __attribute__((address(0x7E3D)));


extern volatile __bit I2C_READ __attribute__((address(0x7E3A)));


extern volatile __bit I2C_START __attribute__((address(0x7E3B)));


extern volatile __bit I2C_STOP __attribute__((address(0x7E3C)));


extern volatile __bit ICODE20 __attribute__((address(0x78F1)));


extern volatile __bit ICODE21 __attribute__((address(0x78F2)));


extern volatile __bit IDISSEN __attribute__((address(0x7AA9)));


extern volatile __bit IDLEN __attribute__((address(0x7E9F)));


extern volatile __bit INT0E __attribute__((address(0x7F94)));


extern volatile __bit INT0F __attribute__((address(0x7F91)));


extern volatile __bit INT0IE __attribute__((address(0x7F94)));


extern volatile __bit INT0IF __attribute__((address(0x7F91)));


extern volatile __bit INT1E __attribute__((address(0x7F83)));


extern volatile __bit INT1F __attribute__((address(0x7F80)));


extern volatile __bit INT1IE __attribute__((address(0x7F83)));


extern volatile __bit INT1IF __attribute__((address(0x7F80)));


extern volatile __bit INT1IP __attribute__((address(0x7F86)));


extern volatile __bit INT1P __attribute__((address(0x7F86)));


extern volatile __bit INT2E __attribute__((address(0x7F84)));


extern volatile __bit INT2F __attribute__((address(0x7F81)));


extern volatile __bit INT2IE __attribute__((address(0x7F84)));


extern volatile __bit INT2IF __attribute__((address(0x7F81)));


extern volatile __bit INT2IP __attribute__((address(0x7F87)));


extern volatile __bit INT2P __attribute__((address(0x7F87)));


extern volatile __bit INT3E __attribute__((address(0x7F85)));


extern volatile __bit INT3F __attribute__((address(0x7F82)));


extern volatile __bit INT3IE __attribute__((address(0x7F85)));


extern volatile __bit INT3IF __attribute__((address(0x7F82)));


extern volatile __bit INT3IP __attribute__((address(0x7F89)));


extern volatile __bit INT3P __attribute__((address(0x7F89)));


extern volatile __bit INTEDG0 __attribute__((address(0x7F8E)));


extern volatile __bit INTEDG1 __attribute__((address(0x7F8D)));


extern volatile __bit INTEDG2 __attribute__((address(0x7F8C)));


extern volatile __bit INTEDG3 __attribute__((address(0x7F8B)));


extern volatile __bit INTSRC __attribute__((address(0x7CDF)));


extern volatile __bit IOCB4 __attribute__((address(0x7AD4)));


extern volatile __bit IOCB5 __attribute__((address(0x7AD5)));


extern volatile __bit IOCB6 __attribute__((address(0x7AD6)));


extern volatile __bit IOCB7 __attribute__((address(0x7AD7)));


extern volatile __bit IPEN __attribute__((address(0x7E87)));


extern volatile __bit IRCF0 __attribute__((address(0x7E9C)));


extern volatile __bit IRCF1 __attribute__((address(0x7E9D)));


extern volatile __bit IRCF2 __attribute__((address(0x7E9E)));


extern volatile __bit IRNG0 __attribute__((address(0x7A98)));


extern volatile __bit IRNG1 __attribute__((address(0x7A99)));


extern volatile __bit IRVST __attribute__((address(0x7D45)));


extern volatile __bit IRXIE __attribute__((address(0x7BB7)));


extern volatile __bit IRXIF __attribute__((address(0x7BBF)));


extern volatile __bit IRXIP __attribute__((address(0x7BC7)));


extern volatile __bit ITRIM0 __attribute__((address(0x7A9A)));


extern volatile __bit ITRIM1 __attribute__((address(0x7A9B)));


extern volatile __bit ITRIM2 __attribute__((address(0x7A9C)));


extern volatile __bit ITRIM3 __attribute__((address(0x7A9D)));


extern volatile __bit ITRIM4 __attribute__((address(0x7A9E)));


extern volatile __bit ITRIM5 __attribute__((address(0x7A9F)));


extern volatile __bit JTOFF __attribute__((address(0x7B01)));


extern volatile __bit JTOFF_FILHIT1 __attribute__((address(0x7B01)));


extern volatile __bit LA0 __attribute__((address(0x7C48)));


extern volatile __bit LA1 __attribute__((address(0x7C49)));


extern volatile __bit LA2 __attribute__((address(0x7C4A)));


extern volatile __bit LA3 __attribute__((address(0x7C4B)));


extern volatile __bit LA5 __attribute__((address(0x7C4D)));


extern volatile __bit LA6 __attribute__((address(0x7C4E)));


extern volatile __bit LA7 __attribute__((address(0x7C4F)));


extern volatile __bit LATA0 __attribute__((address(0x7C48)));


extern volatile __bit LATA1 __attribute__((address(0x7C49)));


extern volatile __bit LATA2 __attribute__((address(0x7C4A)));


extern volatile __bit LATA3 __attribute__((address(0x7C4B)));


extern volatile __bit LATA5 __attribute__((address(0x7C4D)));


extern volatile __bit LATA6 __attribute__((address(0x7C4E)));


extern volatile __bit LATA7 __attribute__((address(0x7C4F)));


extern volatile __bit LATB0 __attribute__((address(0x7C50)));


extern volatile __bit LATB1 __attribute__((address(0x7C51)));


extern volatile __bit LATB2 __attribute__((address(0x7C52)));


extern volatile __bit LATB3 __attribute__((address(0x7C53)));


extern volatile __bit LATB4 __attribute__((address(0x7C54)));


extern volatile __bit LATB5 __attribute__((address(0x7C55)));


extern volatile __bit LATB6 __attribute__((address(0x7C56)));


extern volatile __bit LATB7 __attribute__((address(0x7C57)));


extern volatile __bit LATC0 __attribute__((address(0x7C58)));


extern volatile __bit LATC1 __attribute__((address(0x7C59)));


extern volatile __bit LATC2 __attribute__((address(0x7C5A)));


extern volatile __bit LATC3 __attribute__((address(0x7C5B)));


extern volatile __bit LATC4 __attribute__((address(0x7C5C)));


extern volatile __bit LATC5 __attribute__((address(0x7C5D)));


extern volatile __bit LATC6 __attribute__((address(0x7C5E)));


extern volatile __bit LATC7 __attribute__((address(0x7C5F)));


extern volatile __bit LB0 __attribute__((address(0x7C50)));


extern volatile __bit LB1 __attribute__((address(0x7C51)));


extern volatile __bit LB2 __attribute__((address(0x7C52)));


extern volatile __bit LB3 __attribute__((address(0x7C53)));


extern volatile __bit LB4 __attribute__((address(0x7C54)));


extern volatile __bit LB5 __attribute__((address(0x7C55)));


extern volatile __bit LB6 __attribute__((address(0x7C56)));


extern volatile __bit LB7 __attribute__((address(0x7C57)));


extern volatile __bit LC0 __attribute__((address(0x7C58)));


extern volatile __bit LC1 __attribute__((address(0x7C59)));


extern volatile __bit LC2 __attribute__((address(0x7C5A)));


extern volatile __bit LC3 __attribute__((address(0x7C5B)));


extern volatile __bit LC4 __attribute__((address(0x7C5C)));


extern volatile __bit LC5 __attribute__((address(0x7C5D)));


extern volatile __bit LC6 __attribute__((address(0x7C5E)));


extern volatile __bit LC7 __attribute__((address(0x7C5F)));


extern volatile __bit LVDIE __attribute__((address(0x7D02)));


extern volatile __bit LVDIF __attribute__((address(0x7D0A)));


extern volatile __bit LVDIN __attribute__((address(0x7C05)));


extern volatile __bit LVDIP __attribute__((address(0x7D12)));


extern volatile __bit LVDL0 __attribute__((address(0x7E90)));


extern volatile __bit LVDL1 __attribute__((address(0x7E91)));


extern volatile __bit LVDL2 __attribute__((address(0x7E92)));


extern volatile __bit LVDL3 __attribute__((address(0x7E93)));


extern volatile __bit MDSEL0 __attribute__((address(0x7B96)));


extern volatile __bit MDSEL1 __attribute__((address(0x7B97)));


extern volatile __bit MFIOFS __attribute__((address(0x7E91)));


extern volatile __bit MFIOSEL __attribute__((address(0x7E90)));


extern volatile __bit MSK0 __attribute__((address(0x7E40)));


extern volatile __bit MSK1 __attribute__((address(0x7E41)));


extern volatile __bit MSK2 __attribute__((address(0x7E42)));


extern volatile __bit MSK3 __attribute__((address(0x7E43)));


extern volatile __bit MSK4 __attribute__((address(0x7E44)));


extern volatile __bit MSK5 __attribute__((address(0x7E45)));


extern volatile __bit MSK6 __attribute__((address(0x7E46)));


extern volatile __bit MSK7 __attribute__((address(0x7E47)));


extern volatile __bit NEGATIVE __attribute__((address(0x7EC4)));


extern volatile __bit NOT_A __attribute__((address(0x7E3D)));


extern volatile __bit NOT_ADDRESS __attribute__((address(0x7E3D)));


extern volatile __bit NOT_BOR __attribute__((address(0x7E80)));


extern volatile __bit NOT_CM __attribute__((address(0x7E85)));


extern volatile __bit NOT_DONE __attribute__((address(0x7E11)));


extern volatile __bit NOT_FIFOEMPTY __attribute__((address(0x7B8F)));


extern volatile __bit NOT_PD __attribute__((address(0x7E82)));


extern volatile __bit NOT_POR __attribute__((address(0x7E81)));


extern volatile __bit NOT_RBPU __attribute__((address(0x7F8F)));


extern volatile __bit NOT_RC8 __attribute__((address(0x7D5E)));


extern volatile __bit NOT_RI __attribute__((address(0x7E84)));


extern volatile __bit NOT_T1DONE __attribute__((address(0x7D53)));


extern volatile __bit NOT_T1SYNC __attribute__((address(0x7E6A)));


extern volatile __bit NOT_T3DONE __attribute__((address(0x7D83)));


extern volatile __bit NOT_T3SYNC __attribute__((address(0x7D8A)));


extern volatile __bit NOT_TO __attribute__((address(0x7E83)));


extern volatile __bit NOT_TX8 __attribute__((address(0x7D66)));


extern volatile __bit NOT_W __attribute__((address(0x7E3A)));


extern volatile __bit NOT_WRITE __attribute__((address(0x7E3A)));


extern volatile __bit OERR1 __attribute__((address(0x7D59)));


extern volatile __bit OERR2 __attribute__((address(0x7D31)));


extern volatile __bit OSCFIE __attribute__((address(0x7D07)));


extern volatile __bit OSCFIF __attribute__((address(0x7D0F)));


extern volatile __bit OSCFIP __attribute__((address(0x7D17)));


extern volatile __bit OSTS __attribute__((address(0x7E9B)));


extern volatile __bit OV __attribute__((address(0x7EC3)));


extern volatile __bit OVERFLOW __attribute__((address(0x7EC3)));


extern volatile __bit P1DC0 __attribute__((address(0x7DF0)));


extern volatile __bit P1DC1 __attribute__((address(0x7DF1)));


extern volatile __bit P1DC2 __attribute__((address(0x7DF2)));


extern volatile __bit P1DC3 __attribute__((address(0x7DF3)));


extern volatile __bit P1DC4 __attribute__((address(0x7DF4)));


extern volatile __bit P1DC5 __attribute__((address(0x7DF5)));


extern volatile __bit P1DC6 __attribute__((address(0x7DF6)));


extern volatile __bit P1M0 __attribute__((address(0x7DDE)));


extern volatile __bit P1M1 __attribute__((address(0x7DDF)));


extern volatile __bit P1RSEN __attribute__((address(0x7DF7)));


extern volatile __bit PA1 __attribute__((address(0x7C12)));


extern volatile __bit PA2 __attribute__((address(0x7C11)));


extern volatile __bit PC3E __attribute__((address(0x7C23)));


extern volatile __bit PCFG0 __attribute__((address(0x7AE8)));


extern volatile __bit PCFG1 __attribute__((address(0x7AE9)));


extern volatile __bit PCFG10 __attribute__((address(0x7AE2)));


extern volatile __bit PCFG2 __attribute__((address(0x7AEA)));


extern volatile __bit PCFG3 __attribute__((address(0x7AEB)));


extern volatile __bit PCFG4 __attribute__((address(0x7AEC)));


extern volatile __bit PCFG8 __attribute__((address(0x7AE0)));


extern volatile __bit PCFG9 __attribute__((address(0x7AE1)));


extern volatile __bit PD __attribute__((address(0x7E82)));


extern volatile __bit PEIE __attribute__((address(0x7F96)));


extern volatile __bit PEIE_GIEL __attribute__((address(0x7F96)));


extern volatile __bit PEN __attribute__((address(0x7E2A)));


extern volatile __bit PLLEN __attribute__((address(0x7CDE)));


extern volatile __bit PMPTTL __attribute__((address(0x7AB0)));


extern volatile __bit POR __attribute__((address(0x7E81)));


extern volatile __bit PRSEG0 __attribute__((address(0x7220)));


extern volatile __bit PRSEG1 __attribute__((address(0x7221)));


extern volatile __bit PRSEG2 __attribute__((address(0x7222)));


extern volatile __bit PSA __attribute__((address(0x7EAB)));


extern volatile __bit PSPMD __attribute__((address(0x7AC7)));


extern volatile __bit PSS1AC0 __attribute__((address(0x7DFA)));


extern volatile __bit PSS1AC1 __attribute__((address(0x7DFB)));


extern volatile __bit PSS1BD0 __attribute__((address(0x7DF8)));


extern volatile __bit PSS1BD1 __attribute__((address(0x7DF9)));


extern volatile __bit RA0 __attribute__((address(0x7C00)));


extern volatile __bit RA1 __attribute__((address(0x7C01)));


extern volatile __bit RA2 __attribute__((address(0x7C02)));


extern volatile __bit RA3 __attribute__((address(0x7C03)));


extern volatile __bit RA5 __attribute__((address(0x7C05)));


extern volatile __bit RA6 __attribute__((address(0x7C06)));


extern volatile __bit RA7 __attribute__((address(0x7C07)));


extern volatile __bit RB0DBEN __attribute__((address(0x7B02)));


extern volatile __bit RB0DBEN_FILHIT2 __attribute__((address(0x7B02)));


extern volatile __bit RB2 __attribute__((address(0x7C0A)));


extern volatile __bit RB3 __attribute__((address(0x7C0B)));


extern volatile __bit RB4 __attribute__((address(0x7C0C)));


extern volatile __bit RB5 __attribute__((address(0x7C0D)));


extern volatile __bit RB6 __attribute__((address(0x7C0E)));


extern volatile __bit RB7 __attribute__((address(0x7C0F)));


extern volatile __bit RBIE __attribute__((address(0x7F93)));


extern volatile __bit RBIF __attribute__((address(0x7F90)));


extern volatile __bit RBIP __attribute__((address(0x7F88)));


extern volatile __bit RBPU __attribute__((address(0x7F8F)));


extern volatile __bit RC0 __attribute__((address(0x7C10)));


extern volatile __bit RC1 __attribute__((address(0x7C11)));


extern volatile __bit RC1IE __attribute__((address(0x7CED)));


extern volatile __bit RC1IF __attribute__((address(0x7CF5)));


extern volatile __bit RC1IP __attribute__((address(0x7CFD)));


extern volatile __bit RC2 __attribute__((address(0x7C12)));


extern volatile __bit RC2IE __attribute__((address(0x7D1D)));


extern volatile __bit RC2IF __attribute__((address(0x7D25)));


extern volatile __bit RC2IP __attribute__((address(0x7D2D)));


extern volatile __bit RC3 __attribute__((address(0x7C13)));


extern volatile __bit RC4 __attribute__((address(0x7C14)));


extern volatile __bit RC5 __attribute__((address(0x7C15)));


extern volatile __bit RC6 __attribute__((address(0x7C16)));


extern volatile __bit RC7 __attribute__((address(0x7C17)));


extern volatile __bit RC8_9 __attribute__((address(0x7D5E)));


extern volatile __bit RC8_92 __attribute__((address(0x7D36)));


extern volatile __bit RC9 __attribute__((address(0x7D5E)));


extern volatile __bit RC92 __attribute__((address(0x7D36)));


extern volatile __bit RCD8 __attribute__((address(0x7D58)));


extern volatile __bit RCD82 __attribute__((address(0x7D30)));


extern volatile __bit RCEN __attribute__((address(0x7E2B)));


extern volatile __bit RCIDL1 __attribute__((address(0x7D3E)));


extern volatile __bit RCIDL2 __attribute__((address(0x7DCE)));


extern volatile __bit RCIE __attribute__((address(0x7CED)));


extern volatile __bit RCIF __attribute__((address(0x7CF5)));


extern volatile __bit RCIP __attribute__((address(0x7CFD)));


extern volatile __bit RCMT1 __attribute__((address(0x7D3E)));


extern volatile __bit RCMT2 __attribute__((address(0x7DCE)));


extern volatile __bit RD __attribute__((address(0x7BF8)));


extern volatile __bit RD163 __attribute__((address(0x7D8F)));


extern volatile __bit RE3 __attribute__((address(0x7C23)));


extern volatile __bit READ_WRITE __attribute__((address(0x7E3A)));


extern volatile __bit REC0 __attribute__((address(0x7208)));


extern volatile __bit REC1 __attribute__((address(0x7209)));


extern volatile __bit REC2 __attribute__((address(0x720A)));


extern volatile __bit REC3 __attribute__((address(0x720B)));


extern volatile __bit REC4 __attribute__((address(0x720C)));


extern volatile __bit REC5 __attribute__((address(0x720D)));


extern volatile __bit REC6 __attribute__((address(0x720E)));


extern volatile __bit REC7 __attribute__((address(0x720F)));


extern volatile __bit REGSLP __attribute__((address(0x7E8F)));


extern volatile __bit RI __attribute__((address(0x7E84)));


extern volatile __bit RJPU __attribute__((address(0x7C07)));


extern volatile __bit RODIV0 __attribute__((address(0x7CD0)));


extern volatile __bit RODIV1 __attribute__((address(0x7CD1)));


extern volatile __bit RODIV2 __attribute__((address(0x7CD2)));


extern volatile __bit RODIV3 __attribute__((address(0x7CD3)));


extern volatile __bit ROON __attribute__((address(0x7CD7)));


extern volatile __bit ROSEL __attribute__((address(0x7CD4)));


extern volatile __bit ROSSLP __attribute__((address(0x7CD5)));


extern volatile __bit RSEN __attribute__((address(0x7E29)));


extern volatile __bit RW __attribute__((address(0x7E3A)));


extern volatile __bit RX91 __attribute__((address(0x7D5E)));


extern volatile __bit RX92 __attribute__((address(0x7D36)));


extern volatile __bit RX9D1 __attribute__((address(0x7D58)));


extern volatile __bit RX9D2 __attribute__((address(0x7D30)));


extern volatile __bit RXB0D00 __attribute__((address(0x7B30)));


extern volatile __bit RXB0D01 __attribute__((address(0x7B31)));


extern volatile __bit RXB0D02 __attribute__((address(0x7B32)));


extern volatile __bit RXB0D03 __attribute__((address(0x7B33)));


extern volatile __bit RXB0D04 __attribute__((address(0x7B34)));


extern volatile __bit RXB0D05 __attribute__((address(0x7B35)));


extern volatile __bit RXB0D06 __attribute__((address(0x7B36)));


extern volatile __bit RXB0D07 __attribute__((address(0x7B37)));


extern volatile __bit RXB0D10 __attribute__((address(0x7B38)));


extern volatile __bit RXB0D11 __attribute__((address(0x7B39)));


extern volatile __bit RXB0D12 __attribute__((address(0x7B3A)));


extern volatile __bit RXB0D13 __attribute__((address(0x7B3B)));


extern volatile __bit RXB0D14 __attribute__((address(0x7B3C)));


extern volatile __bit RXB0D15 __attribute__((address(0x7B3D)));


extern volatile __bit RXB0D16 __attribute__((address(0x7B3E)));


extern volatile __bit RXB0D17 __attribute__((address(0x7B3F)));


extern volatile __bit RXB0D20 __attribute__((address(0x7B40)));


extern volatile __bit RXB0D21 __attribute__((address(0x7B41)));


extern volatile __bit RXB0D22 __attribute__((address(0x7B42)));


extern volatile __bit RXB0D23 __attribute__((address(0x7B43)));


extern volatile __bit RXB0D24 __attribute__((address(0x7B44)));


extern volatile __bit RXB0D25 __attribute__((address(0x7B45)));


extern volatile __bit RXB0D26 __attribute__((address(0x7B46)));


extern volatile __bit RXB0D27 __attribute__((address(0x7B47)));


extern volatile __bit RXB0D30 __attribute__((address(0x7B48)));


extern volatile __bit RXB0D31 __attribute__((address(0x7B49)));


extern volatile __bit RXB0D32 __attribute__((address(0x7B4A)));


extern volatile __bit RXB0D33 __attribute__((address(0x7B4B)));


extern volatile __bit RXB0D34 __attribute__((address(0x7B4C)));


extern volatile __bit RXB0D35 __attribute__((address(0x7B4D)));


extern volatile __bit RXB0D36 __attribute__((address(0x7B4E)));


extern volatile __bit RXB0D37 __attribute__((address(0x7B4F)));


extern volatile __bit RXB0D40 __attribute__((address(0x7B50)));


extern volatile __bit RXB0D41 __attribute__((address(0x7B51)));


extern volatile __bit RXB0D42 __attribute__((address(0x7B52)));


extern volatile __bit RXB0D43 __attribute__((address(0x7B53)));


extern volatile __bit RXB0D44 __attribute__((address(0x7B54)));


extern volatile __bit RXB0D45 __attribute__((address(0x7B55)));


extern volatile __bit RXB0D46 __attribute__((address(0x7B56)));


extern volatile __bit RXB0D47 __attribute__((address(0x7B57)));


extern volatile __bit RXB0D50 __attribute__((address(0x7B58)));


extern volatile __bit RXB0D51 __attribute__((address(0x7B59)));


extern volatile __bit RXB0D52 __attribute__((address(0x7B5A)));


extern volatile __bit RXB0D53 __attribute__((address(0x7B5B)));


extern volatile __bit RXB0D54 __attribute__((address(0x7B5C)));


extern volatile __bit RXB0D55 __attribute__((address(0x7B5D)));


extern volatile __bit RXB0D56 __attribute__((address(0x7B5E)));


extern volatile __bit RXB0D57 __attribute__((address(0x7B5F)));


extern volatile __bit RXB0D60 __attribute__((address(0x7B60)));


extern volatile __bit RXB0D61 __attribute__((address(0x7B61)));


extern volatile __bit RXB0D62 __attribute__((address(0x7B62)));


extern volatile __bit RXB0D63 __attribute__((address(0x7B63)));


extern volatile __bit RXB0D64 __attribute__((address(0x7B64)));


extern volatile __bit RXB0D65 __attribute__((address(0x7B65)));


extern volatile __bit RXB0D66 __attribute__((address(0x7B66)));


extern volatile __bit RXB0D67 __attribute__((address(0x7B67)));


extern volatile __bit RXB0D70 __attribute__((address(0x7B68)));


extern volatile __bit RXB0D71 __attribute__((address(0x7B69)));


extern volatile __bit RXB0D72 __attribute__((address(0x7B6A)));


extern volatile __bit RXB0D73 __attribute__((address(0x7B6B)));


extern volatile __bit RXB0D74 __attribute__((address(0x7B6C)));


extern volatile __bit RXB0D75 __attribute__((address(0x7B6D)));


extern volatile __bit RXB0D76 __attribute__((address(0x7B6E)));


extern volatile __bit RXB0D77 __attribute__((address(0x7B6F)));


extern volatile __bit RXB0DLC0 __attribute__((address(0x7B28)));


extern volatile __bit RXB0DLC1 __attribute__((address(0x7B29)));


extern volatile __bit RXB0DLC2 __attribute__((address(0x7B2A)));


extern volatile __bit RXB0DLC3 __attribute__((address(0x7B2B)));


extern volatile __bit RXB0EID0 __attribute__((address(0x7B20)));


extern volatile __bit RXB0EID1 __attribute__((address(0x7B21)));


extern volatile __bit RXB0EID10 __attribute__((address(0x7B1A)));


extern volatile __bit RXB0EID11 __attribute__((address(0x7B1B)));


extern volatile __bit RXB0EID12 __attribute__((address(0x7B1C)));


extern volatile __bit RXB0EID13 __attribute__((address(0x7B1D)));


extern volatile __bit RXB0EID14 __attribute__((address(0x7B1E)));


extern volatile __bit RXB0EID15 __attribute__((address(0x7B1F)));


extern volatile __bit RXB0EID16 __attribute__((address(0x7B10)));


extern volatile __bit RXB0EID17 __attribute__((address(0x7B11)));


extern volatile __bit RXB0EID2 __attribute__((address(0x7B22)));


extern volatile __bit RXB0EID3 __attribute__((address(0x7B23)));


extern volatile __bit RXB0EID4 __attribute__((address(0x7B24)));


extern volatile __bit RXB0EID5 __attribute__((address(0x7B25)));


extern volatile __bit RXB0EID6 __attribute__((address(0x7B26)));


extern volatile __bit RXB0EID7 __attribute__((address(0x7B27)));


extern volatile __bit RXB0EID8 __attribute__((address(0x7B18)));


extern volatile __bit RXB0EID9 __attribute__((address(0x7B19)));


extern volatile __bit RXB0EXID __attribute__((address(0x7B13)));


extern volatile __bit RXB0FILHIT0 __attribute__((address(0x7B00)));


extern volatile __bit RXB0FILHIT1 __attribute__((address(0x7B01)));


extern volatile __bit RXB0FILHIT2 __attribute__((address(0x7B02)));


extern volatile __bit RXB0FILHIT3 __attribute__((address(0x7B03)));


extern volatile __bit RXB0FILHIT4 __attribute__((address(0x7B04)));


extern volatile __bit RXB0FUL __attribute__((address(0x7B07)));


extern volatile __bit RXB0IF __attribute__((address(0x7BB8)));


extern volatile __bit RXB0IP __attribute__((address(0x7BC0)));


extern volatile __bit RXB0M0 __attribute__((address(0x7B05)));


extern volatile __bit RXB0M1 __attribute__((address(0x7B06)));


extern volatile __bit RXB0OVFL __attribute__((address(0x7B8F)));


extern volatile __bit RXB0RB0 __attribute__((address(0x7B2C)));


extern volatile __bit RXB0RB1 __attribute__((address(0x7B2D)));


extern volatile __bit RXB0RTR __attribute__((address(0x7B2E)));


extern volatile __bit RXB0RTRR0 __attribute__((address(0x7B03)));


extern volatile __bit RXB0RTRRO __attribute__((address(0x7B05)));


extern volatile __bit RXB0SID0 __attribute__((address(0x7B15)));


extern volatile __bit RXB0SID1 __attribute__((address(0x7B16)));


extern volatile __bit RXB0SID10 __attribute__((address(0x7B0F)));


extern volatile __bit RXB0SID2 __attribute__((address(0x7B17)));


extern volatile __bit RXB0SID3 __attribute__((address(0x7B08)));


extern volatile __bit RXB0SID4 __attribute__((address(0x7B09)));


extern volatile __bit RXB0SID5 __attribute__((address(0x7B0A)));


extern volatile __bit RXB0SID6 __attribute__((address(0x7B0B)));


extern volatile __bit RXB0SID7 __attribute__((address(0x7B0C)));


extern volatile __bit RXB0SID8 __attribute__((address(0x7B0D)));


extern volatile __bit RXB0SID9 __attribute__((address(0x7B0E)));


extern volatile __bit RXB0SRR __attribute__((address(0x7B14)));


extern volatile __bit RXB1D00 __attribute__((address(0x79B0)));


extern volatile __bit RXB1D01 __attribute__((address(0x79B1)));


extern volatile __bit RXB1D02 __attribute__((address(0x79B2)));


extern volatile __bit RXB1D03 __attribute__((address(0x79B3)));


extern volatile __bit RXB1D04 __attribute__((address(0x79B4)));


extern volatile __bit RXB1D05 __attribute__((address(0x79B5)));


extern volatile __bit RXB1D06 __attribute__((address(0x79B6)));


extern volatile __bit RXB1D07 __attribute__((address(0x79B7)));


extern volatile __bit RXB1D10 __attribute__((address(0x79B8)));


extern volatile __bit RXB1D11 __attribute__((address(0x79B9)));


extern volatile __bit RXB1D12 __attribute__((address(0x79BA)));


extern volatile __bit RXB1D13 __attribute__((address(0x79BB)));


extern volatile __bit RXB1D14 __attribute__((address(0x79BC)));


extern volatile __bit RXB1D15 __attribute__((address(0x79BD)));


extern volatile __bit RXB1D16 __attribute__((address(0x79BE)));


extern volatile __bit RXB1D17 __attribute__((address(0x79BF)));


extern volatile __bit RXB1D20 __attribute__((address(0x79C0)));


extern volatile __bit RXB1D21 __attribute__((address(0x79C1)));


extern volatile __bit RXB1D22 __attribute__((address(0x79C2)));


extern volatile __bit RXB1D23 __attribute__((address(0x79C3)));


extern volatile __bit RXB1D24 __attribute__((address(0x79C4)));


extern volatile __bit RXB1D25 __attribute__((address(0x79C5)));


extern volatile __bit RXB1D26 __attribute__((address(0x79C6)));


extern volatile __bit RXB1D27 __attribute__((address(0x79C7)));


extern volatile __bit RXB1D30 __attribute__((address(0x79C8)));


extern volatile __bit RXB1D31 __attribute__((address(0x79C9)));


extern volatile __bit RXB1D32 __attribute__((address(0x79CA)));


extern volatile __bit RXB1D33 __attribute__((address(0x79CB)));


extern volatile __bit RXB1D34 __attribute__((address(0x79CC)));


extern volatile __bit RXB1D35 __attribute__((address(0x79CD)));


extern volatile __bit RXB1D36 __attribute__((address(0x79CE)));


extern volatile __bit RXB1D37 __attribute__((address(0x79CF)));


extern volatile __bit RXB1D40 __attribute__((address(0x79D0)));


extern volatile __bit RXB1D41 __attribute__((address(0x79D1)));


extern volatile __bit RXB1D42 __attribute__((address(0x79D2)));


extern volatile __bit RXB1D43 __attribute__((address(0x79D3)));


extern volatile __bit RXB1D44 __attribute__((address(0x79D4)));


extern volatile __bit RXB1D45 __attribute__((address(0x79D5)));


extern volatile __bit RXB1D46 __attribute__((address(0x79D6)));


extern volatile __bit RXB1D47 __attribute__((address(0x79D7)));


extern volatile __bit RXB1D50 __attribute__((address(0x79D8)));


extern volatile __bit RXB1D51 __attribute__((address(0x79D9)));


extern volatile __bit RXB1D52 __attribute__((address(0x79DA)));


extern volatile __bit RXB1D53 __attribute__((address(0x79DB)));


extern volatile __bit RXB1D54 __attribute__((address(0x79DC)));


extern volatile __bit RXB1D55 __attribute__((address(0x79DD)));


extern volatile __bit RXB1D56 __attribute__((address(0x79DE)));


extern volatile __bit RXB1D57 __attribute__((address(0x79DF)));


extern volatile __bit RXB1D60 __attribute__((address(0x79E0)));


extern volatile __bit RXB1D61 __attribute__((address(0x79E1)));


extern volatile __bit RXB1D62 __attribute__((address(0x79E2)));


extern volatile __bit RXB1D63 __attribute__((address(0x79E3)));


extern volatile __bit RXB1D64 __attribute__((address(0x79E4)));


extern volatile __bit RXB1D65 __attribute__((address(0x79E5)));


extern volatile __bit RXB1D66 __attribute__((address(0x79E6)));


extern volatile __bit RXB1D67 __attribute__((address(0x79E7)));


extern volatile __bit RXB1D70 __attribute__((address(0x79E8)));


extern volatile __bit RXB1D71 __attribute__((address(0x79E9)));


extern volatile __bit RXB1D72 __attribute__((address(0x79EA)));


extern volatile __bit RXB1D73 __attribute__((address(0x79EB)));


extern volatile __bit RXB1D74 __attribute__((address(0x79EC)));


extern volatile __bit RXB1D75 __attribute__((address(0x79ED)));


extern volatile __bit RXB1D76 __attribute__((address(0x79EE)));


extern volatile __bit RXB1D77 __attribute__((address(0x79EF)));


extern volatile __bit RXB1DLC0 __attribute__((address(0x79A8)));


extern volatile __bit RXB1DLC1 __attribute__((address(0x79A9)));


extern volatile __bit RXB1DLC2 __attribute__((address(0x79AA)));


extern volatile __bit RXB1DLC3 __attribute__((address(0x79AB)));


extern volatile __bit RXB1EID0 __attribute__((address(0x79A0)));


extern volatile __bit RXB1EID1 __attribute__((address(0x79A1)));


extern volatile __bit RXB1EID10 __attribute__((address(0x799A)));


extern volatile __bit RXB1EID11 __attribute__((address(0x799B)));


extern volatile __bit RXB1EID12 __attribute__((address(0x799C)));


extern volatile __bit RXB1EID13 __attribute__((address(0x799D)));


extern volatile __bit RXB1EID14 __attribute__((address(0x799E)));


extern volatile __bit RXB1EID15 __attribute__((address(0x799F)));


extern volatile __bit RXB1EID16 __attribute__((address(0x7990)));


extern volatile __bit RXB1EID17 __attribute__((address(0x7991)));


extern volatile __bit RXB1EID2 __attribute__((address(0x79A2)));


extern volatile __bit RXB1EID3 __attribute__((address(0x79A3)));


extern volatile __bit RXB1EID4 __attribute__((address(0x79A4)));


extern volatile __bit RXB1EID5 __attribute__((address(0x79A5)));


extern volatile __bit RXB1EID6 __attribute__((address(0x79A6)));


extern volatile __bit RXB1EID7 __attribute__((address(0x79A7)));


extern volatile __bit RXB1EID8 __attribute__((address(0x7998)));


extern volatile __bit RXB1EID9 __attribute__((address(0x7999)));


extern volatile __bit RXB1EXID __attribute__((address(0x7993)));


extern volatile __bit RXB1FILHIT0 __attribute__((address(0x7980)));


extern volatile __bit RXB1FILHIT1 __attribute__((address(0x7981)));


extern volatile __bit RXB1FILHIT2 __attribute__((address(0x7982)));


extern volatile __bit RXB1FILHIT3 __attribute__((address(0x7983)));


extern volatile __bit RXB1FILHIT4 __attribute__((address(0x7984)));


extern volatile __bit RXB1FUL __attribute__((address(0x7987)));


extern volatile __bit RXB1IF __attribute__((address(0x7BB9)));


extern volatile __bit RXB1IP __attribute__((address(0x7BC1)));


extern volatile __bit RXB1M0 __attribute__((address(0x7985)));


extern volatile __bit RXB1M1 __attribute__((address(0x7986)));


extern volatile __bit RXB1OVFL __attribute__((address(0x7B8E)));


extern volatile __bit RXB1RB0 __attribute__((address(0x79AC)));


extern volatile __bit RXB1RB1 __attribute__((address(0x79AD)));


extern volatile __bit RXB1RTR __attribute__((address(0x79AE)));


extern volatile __bit RXB1RTRR0 __attribute__((address(0x7983)));


extern volatile __bit RXB1RTRRO __attribute__((address(0x7985)));


extern volatile __bit RXB1SID0 __attribute__((address(0x7995)));


extern volatile __bit RXB1SID1 __attribute__((address(0x7996)));


extern volatile __bit RXB1SID10 __attribute__((address(0x798F)));


extern volatile __bit RXB1SID2 __attribute__((address(0x7997)));


extern volatile __bit RXB1SID3 __attribute__((address(0x7988)));


extern volatile __bit RXB1SID4 __attribute__((address(0x7989)));


extern volatile __bit RXB1SID5 __attribute__((address(0x798A)));


extern volatile __bit RXB1SID6 __attribute__((address(0x798B)));


extern volatile __bit RXB1SID7 __attribute__((address(0x798C)));


extern volatile __bit RXB1SID8 __attribute__((address(0x798D)));


extern volatile __bit RXB1SID9 __attribute__((address(0x798E)));


extern volatile __bit RXB1SRR __attribute__((address(0x7994)));


extern volatile __bit RXBNIE __attribute__((address(0x7D19)));


extern volatile __bit RXBNIF __attribute__((address(0x7D21)));


extern volatile __bit RXBNIP __attribute__((address(0x7D29)));


extern volatile __bit RXBNOVFL __attribute__((address(0x7B8E)));


extern volatile __bit RXBP __attribute__((address(0x7B8B)));


extern volatile __bit RXBnIE __attribute__((address(0x7BB1)));


extern volatile __bit RXBnIF __attribute__((address(0x7BB9)));


extern volatile __bit RXBnIP __attribute__((address(0x7BC1)));


extern volatile __bit RXDTP1 __attribute__((address(0x7D3D)));


extern volatile __bit RXDTP2 __attribute__((address(0x7DCD)));


extern volatile __bit RXF0EID0 __attribute__((address(0x7718)));


extern volatile __bit RXF0EID1 __attribute__((address(0x7719)));


extern volatile __bit RXF0EID10 __attribute__((address(0x7712)));


extern volatile __bit RXF0EID11 __attribute__((address(0x7713)));


extern volatile __bit RXF0EID12 __attribute__((address(0x7714)));


extern volatile __bit RXF0EID13 __attribute__((address(0x7715)));


extern volatile __bit RXF0EID14 __attribute__((address(0x7716)));


extern volatile __bit RXF0EID15 __attribute__((address(0x7717)));


extern volatile __bit RXF0EID16 __attribute__((address(0x7708)));


extern volatile __bit RXF0EID17 __attribute__((address(0x7709)));


extern volatile __bit RXF0EID2 __attribute__((address(0x771A)));


extern volatile __bit RXF0EID3 __attribute__((address(0x771B)));


extern volatile __bit RXF0EID4 __attribute__((address(0x771C)));


extern volatile __bit RXF0EID5 __attribute__((address(0x771D)));


extern volatile __bit RXF0EID6 __attribute__((address(0x771E)));


extern volatile __bit RXF0EID7 __attribute__((address(0x771F)));


extern volatile __bit RXF0EID8 __attribute__((address(0x7710)));


extern volatile __bit RXF0EID9 __attribute__((address(0x7711)));


extern volatile __bit RXF0EN __attribute__((address(0x7230)));


extern volatile __bit RXF0EXIDEN __attribute__((address(0x770B)));


extern volatile __bit RXF0SID0 __attribute__((address(0x770D)));


extern volatile __bit RXF0SID1 __attribute__((address(0x770E)));


extern volatile __bit RXF0SID10 __attribute__((address(0x7707)));


extern volatile __bit RXF0SID2 __attribute__((address(0x770F)));


extern volatile __bit RXF0SID3 __attribute__((address(0x7700)));


extern volatile __bit RXF0SID4 __attribute__((address(0x7701)));


extern volatile __bit RXF0SID5 __attribute__((address(0x7702)));


extern volatile __bit RXF0SID6 __attribute__((address(0x7703)));


extern volatile __bit RXF0SID7 __attribute__((address(0x7704)));


extern volatile __bit RXF0SID8 __attribute__((address(0x7705)));


extern volatile __bit RXF0SID9 __attribute__((address(0x7706)));


extern volatile __bit RXF10EID0 __attribute__((address(0x72D8)));


extern volatile __bit RXF10EID1 __attribute__((address(0x72D9)));


extern volatile __bit RXF10EID10 __attribute__((address(0x72D2)));


extern volatile __bit RXF10EID11 __attribute__((address(0x72D3)));


extern volatile __bit RXF10EID12 __attribute__((address(0x72D4)));


extern volatile __bit RXF10EID13 __attribute__((address(0x72D5)));


extern volatile __bit RXF10EID14 __attribute__((address(0x72D6)));


extern volatile __bit RXF10EID15 __attribute__((address(0x72D7)));


extern volatile __bit RXF10EID16 __attribute__((address(0x72C8)));


extern volatile __bit RXF10EID17 __attribute__((address(0x72C9)));


extern volatile __bit RXF10EID2 __attribute__((address(0x72DA)));


extern volatile __bit RXF10EID3 __attribute__((address(0x72DB)));


extern volatile __bit RXF10EID4 __attribute__((address(0x72DC)));


extern volatile __bit RXF10EID5 __attribute__((address(0x72DD)));


extern volatile __bit RXF10EID6 __attribute__((address(0x72DE)));


extern volatile __bit RXF10EID7 __attribute__((address(0x72DF)));


extern volatile __bit RXF10EID8 __attribute__((address(0x72D0)));


extern volatile __bit RXF10EID9 __attribute__((address(0x72D1)));


extern volatile __bit RXF10EN __attribute__((address(0x723A)));


extern volatile __bit RXF10EXIDEN __attribute__((address(0x72CB)));


extern volatile __bit RXF10SID0 __attribute__((address(0x72CD)));


extern volatile __bit RXF10SID1 __attribute__((address(0x72CE)));


extern volatile __bit RXF10SID10 __attribute__((address(0x72C7)));


extern volatile __bit RXF10SID2 __attribute__((address(0x72CF)));


extern volatile __bit RXF10SID3 __attribute__((address(0x72C0)));


extern volatile __bit RXF10SID4 __attribute__((address(0x72C1)));


extern volatile __bit RXF10SID5 __attribute__((address(0x72C2)));


extern volatile __bit RXF10SID6 __attribute__((address(0x72C3)));


extern volatile __bit RXF10SID7 __attribute__((address(0x72C4)));


extern volatile __bit RXF10SID8 __attribute__((address(0x72C5)));


extern volatile __bit RXF10SID9 __attribute__((address(0x72C6)));


extern volatile __bit RXF11EID0 __attribute__((address(0x72F8)));


extern volatile __bit RXF11EID1 __attribute__((address(0x72F9)));


extern volatile __bit RXF11EID10 __attribute__((address(0x72F2)));


extern volatile __bit RXF11EID11 __attribute__((address(0x72F3)));


extern volatile __bit RXF11EID12 __attribute__((address(0x72F4)));


extern volatile __bit RXF11EID13 __attribute__((address(0x72F5)));


extern volatile __bit RXF11EID14 __attribute__((address(0x72F6)));


extern volatile __bit RXF11EID15 __attribute__((address(0x72F7)));


extern volatile __bit RXF11EID16 __attribute__((address(0x72E8)));


extern volatile __bit RXF11EID17 __attribute__((address(0x72E9)));


extern volatile __bit RXF11EID2 __attribute__((address(0x72FA)));


extern volatile __bit RXF11EID3 __attribute__((address(0x72FB)));


extern volatile __bit RXF11EID4 __attribute__((address(0x72FC)));


extern volatile __bit RXF11EID5 __attribute__((address(0x72FD)));


extern volatile __bit RXF11EID6 __attribute__((address(0x72FE)));


extern volatile __bit RXF11EID7 __attribute__((address(0x72FF)));


extern volatile __bit RXF11EID8 __attribute__((address(0x72F0)));


extern volatile __bit RXF11EID9 __attribute__((address(0x72F1)));


extern volatile __bit RXF11EN __attribute__((address(0x723B)));


extern volatile __bit RXF11EXIDEN __attribute__((address(0x72EB)));


extern volatile __bit RXF11SID0 __attribute__((address(0x72ED)));


extern volatile __bit RXF11SID1 __attribute__((address(0x72EE)));


extern volatile __bit RXF11SID10 __attribute__((address(0x72E7)));


extern volatile __bit RXF11SID2 __attribute__((address(0x72EF)));


extern volatile __bit RXF11SID3 __attribute__((address(0x72E0)));


extern volatile __bit RXF11SID4 __attribute__((address(0x72E1)));


extern volatile __bit RXF11SID5 __attribute__((address(0x72E2)));


extern volatile __bit RXF11SID6 __attribute__((address(0x72E3)));


extern volatile __bit RXF11SID7 __attribute__((address(0x72E4)));


extern volatile __bit RXF11SID8 __attribute__((address(0x72E5)));


extern volatile __bit RXF11SID9 __attribute__((address(0x72E6)));


extern volatile __bit RXF12EID0 __attribute__((address(0x7318)));


extern volatile __bit RXF12EID1 __attribute__((address(0x7319)));


extern volatile __bit RXF12EID10 __attribute__((address(0x7312)));


extern volatile __bit RXF12EID11 __attribute__((address(0x7313)));


extern volatile __bit RXF12EID12 __attribute__((address(0x7314)));


extern volatile __bit RXF12EID13 __attribute__((address(0x7315)));


extern volatile __bit RXF12EID14 __attribute__((address(0x7316)));


extern volatile __bit RXF12EID15 __attribute__((address(0x7317)));


extern volatile __bit RXF12EID16 __attribute__((address(0x7308)));


extern volatile __bit RXF12EID17 __attribute__((address(0x7309)));


extern volatile __bit RXF12EID2 __attribute__((address(0x731A)));


extern volatile __bit RXF12EID3 __attribute__((address(0x731B)));


extern volatile __bit RXF12EID4 __attribute__((address(0x731C)));


extern volatile __bit RXF12EID5 __attribute__((address(0x731D)));


extern volatile __bit RXF12EID6 __attribute__((address(0x731E)));


extern volatile __bit RXF12EID7 __attribute__((address(0x731F)));


extern volatile __bit RXF12EID8 __attribute__((address(0x7310)));


extern volatile __bit RXF12EID9 __attribute__((address(0x7311)));


extern volatile __bit RXF12EN __attribute__((address(0x723C)));


extern volatile __bit RXF12EXIDEN __attribute__((address(0x730B)));


extern volatile __bit RXF12SID0 __attribute__((address(0x730D)));


extern volatile __bit RXF12SID1 __attribute__((address(0x730E)));


extern volatile __bit RXF12SID10 __attribute__((address(0x7307)));


extern volatile __bit RXF12SID2 __attribute__((address(0x730F)));


extern volatile __bit RXF12SID3 __attribute__((address(0x7300)));


extern volatile __bit RXF12SID4 __attribute__((address(0x7301)));


extern volatile __bit RXF12SID5 __attribute__((address(0x7302)));


extern volatile __bit RXF12SID6 __attribute__((address(0x7303)));


extern volatile __bit RXF12SID7 __attribute__((address(0x7304)));


extern volatile __bit RXF12SID8 __attribute__((address(0x7305)));


extern volatile __bit RXF12SID9 __attribute__((address(0x7306)));


extern volatile __bit RXF13EID0 __attribute__((address(0x7338)));


extern volatile __bit RXF13EID1 __attribute__((address(0x7339)));


extern volatile __bit RXF13EID10 __attribute__((address(0x7332)));


extern volatile __bit RXF13EID11 __attribute__((address(0x7333)));


extern volatile __bit RXF13EID12 __attribute__((address(0x7334)));


extern volatile __bit RXF13EID13 __attribute__((address(0x7335)));


extern volatile __bit RXF13EID14 __attribute__((address(0x7336)));


extern volatile __bit RXF13EID15 __attribute__((address(0x7337)));


extern volatile __bit RXF13EID16 __attribute__((address(0x7328)));


extern volatile __bit RXF13EID17 __attribute__((address(0x7329)));


extern volatile __bit RXF13EID2 __attribute__((address(0x733A)));


extern volatile __bit RXF13EID3 __attribute__((address(0x733B)));


extern volatile __bit RXF13EID4 __attribute__((address(0x733C)));


extern volatile __bit RXF13EID5 __attribute__((address(0x733D)));


extern volatile __bit RXF13EID6 __attribute__((address(0x733E)));


extern volatile __bit RXF13EID7 __attribute__((address(0x733F)));


extern volatile __bit RXF13EID8 __attribute__((address(0x7330)));


extern volatile __bit RXF13EID9 __attribute__((address(0x7331)));


extern volatile __bit RXF13EN __attribute__((address(0x723D)));


extern volatile __bit RXF13EXIDEN __attribute__((address(0x732B)));


extern volatile __bit RXF13SID0 __attribute__((address(0x732D)));


extern volatile __bit RXF13SID1 __attribute__((address(0x732E)));


extern volatile __bit RXF13SID10 __attribute__((address(0x7327)));


extern volatile __bit RXF13SID2 __attribute__((address(0x732F)));


extern volatile __bit RXF13SID3 __attribute__((address(0x7320)));


extern volatile __bit RXF13SID4 __attribute__((address(0x7321)));


extern volatile __bit RXF13SID5 __attribute__((address(0x7322)));


extern volatile __bit RXF13SID6 __attribute__((address(0x7323)));


extern volatile __bit RXF13SID7 __attribute__((address(0x7324)));


extern volatile __bit RXF13SID8 __attribute__((address(0x7325)));


extern volatile __bit RXF13SID9 __attribute__((address(0x7326)));


extern volatile __bit RXF14EID0 __attribute__((address(0x7358)));


extern volatile __bit RXF14EID1 __attribute__((address(0x7359)));


extern volatile __bit RXF14EID10 __attribute__((address(0x7352)));


extern volatile __bit RXF14EID11 __attribute__((address(0x7353)));


extern volatile __bit RXF14EID12 __attribute__((address(0x7354)));


extern volatile __bit RXF14EID13 __attribute__((address(0x7355)));


extern volatile __bit RXF14EID14 __attribute__((address(0x7356)));


extern volatile __bit RXF14EID15 __attribute__((address(0x7357)));


extern volatile __bit RXF14EID16 __attribute__((address(0x7348)));


extern volatile __bit RXF14EID17 __attribute__((address(0x7349)));


extern volatile __bit RXF14EID2 __attribute__((address(0x735A)));


extern volatile __bit RXF14EID3 __attribute__((address(0x735B)));


extern volatile __bit RXF14EID4 __attribute__((address(0x735C)));


extern volatile __bit RXF14EID5 __attribute__((address(0x735D)));


extern volatile __bit RXF14EID6 __attribute__((address(0x735E)));


extern volatile __bit RXF14EID7 __attribute__((address(0x735F)));


extern volatile __bit RXF14EID8 __attribute__((address(0x7350)));


extern volatile __bit RXF14EID9 __attribute__((address(0x7351)));


extern volatile __bit RXF14EN __attribute__((address(0x723E)));


extern volatile __bit RXF14EXIDEN __attribute__((address(0x734B)));


extern volatile __bit RXF14SID0 __attribute__((address(0x734D)));


extern volatile __bit RXF14SID1 __attribute__((address(0x734E)));


extern volatile __bit RXF14SID10 __attribute__((address(0x7347)));


extern volatile __bit RXF14SID2 __attribute__((address(0x734F)));


extern volatile __bit RXF14SID3 __attribute__((address(0x7340)));


extern volatile __bit RXF14SID4 __attribute__((address(0x7341)));


extern volatile __bit RXF14SID5 __attribute__((address(0x7342)));


extern volatile __bit RXF14SID6 __attribute__((address(0x7343)));


extern volatile __bit RXF14SID7 __attribute__((address(0x7344)));


extern volatile __bit RXF14SID8 __attribute__((address(0x7345)));


extern volatile __bit RXF14SID9 __attribute__((address(0x7346)));


extern volatile __bit RXF15EID0 __attribute__((address(0x7378)));


extern volatile __bit RXF15EID1 __attribute__((address(0x7379)));


extern volatile __bit RXF15EID10 __attribute__((address(0x7372)));


extern volatile __bit RXF15EID11 __attribute__((address(0x7373)));


extern volatile __bit RXF15EID12 __attribute__((address(0x7374)));


extern volatile __bit RXF15EID13 __attribute__((address(0x7375)));


extern volatile __bit RXF15EID14 __attribute__((address(0x7376)));


extern volatile __bit RXF15EID15 __attribute__((address(0x7377)));


extern volatile __bit RXF15EID16 __attribute__((address(0x7368)));


extern volatile __bit RXF15EID17 __attribute__((address(0x7369)));


extern volatile __bit RXF15EID2 __attribute__((address(0x737A)));


extern volatile __bit RXF15EID3 __attribute__((address(0x737B)));


extern volatile __bit RXF15EID4 __attribute__((address(0x737C)));


extern volatile __bit RXF15EID5 __attribute__((address(0x737D)));


extern volatile __bit RXF15EID6 __attribute__((address(0x737E)));


extern volatile __bit RXF15EID7 __attribute__((address(0x737F)));


extern volatile __bit RXF15EID8 __attribute__((address(0x7370)));


extern volatile __bit RXF15EID9 __attribute__((address(0x7371)));


extern volatile __bit RXF15EN __attribute__((address(0x723F)));


extern volatile __bit RXF15EXIDEN __attribute__((address(0x736B)));


extern volatile __bit RXF15SID0 __attribute__((address(0x736D)));


extern volatile __bit RXF15SID1 __attribute__((address(0x736E)));


extern volatile __bit RXF15SID10 __attribute__((address(0x7367)));


extern volatile __bit RXF15SID2 __attribute__((address(0x736F)));


extern volatile __bit RXF15SID3 __attribute__((address(0x7360)));


extern volatile __bit RXF15SID4 __attribute__((address(0x7361)));


extern volatile __bit RXF15SID5 __attribute__((address(0x7362)));


extern volatile __bit RXF15SID6 __attribute__((address(0x7363)));


extern volatile __bit RXF15SID7 __attribute__((address(0x7364)));


extern volatile __bit RXF15SID8 __attribute__((address(0x7365)));


extern volatile __bit RXF15SID9 __attribute__((address(0x7366)));


extern volatile __bit RXF1EID0 __attribute__((address(0x7738)));


extern volatile __bit RXF1EID1 __attribute__((address(0x7739)));


extern volatile __bit RXF1EID10 __attribute__((address(0x7732)));


extern volatile __bit RXF1EID11 __attribute__((address(0x7733)));


extern volatile __bit RXF1EID12 __attribute__((address(0x7734)));


extern volatile __bit RXF1EID13 __attribute__((address(0x7735)));


extern volatile __bit RXF1EID14 __attribute__((address(0x7736)));


extern volatile __bit RXF1EID15 __attribute__((address(0x7737)));


extern volatile __bit RXF1EID16 __attribute__((address(0x7728)));


extern volatile __bit RXF1EID17 __attribute__((address(0x7729)));


extern volatile __bit RXF1EID2 __attribute__((address(0x773A)));


extern volatile __bit RXF1EID3 __attribute__((address(0x773B)));


extern volatile __bit RXF1EID4 __attribute__((address(0x773C)));


extern volatile __bit RXF1EID5 __attribute__((address(0x773D)));


extern volatile __bit RXF1EID6 __attribute__((address(0x773E)));


extern volatile __bit RXF1EID7 __attribute__((address(0x773F)));


extern volatile __bit RXF1EID8 __attribute__((address(0x7730)));


extern volatile __bit RXF1EID9 __attribute__((address(0x7731)));


extern volatile __bit RXF1EN __attribute__((address(0x7231)));


extern volatile __bit RXF1EXIDEN __attribute__((address(0x772B)));


extern volatile __bit RXF1SID0 __attribute__((address(0x772D)));


extern volatile __bit RXF1SID1 __attribute__((address(0x772E)));


extern volatile __bit RXF1SID10 __attribute__((address(0x7727)));


extern volatile __bit RXF1SID2 __attribute__((address(0x772F)));


extern volatile __bit RXF1SID3 __attribute__((address(0x7720)));


extern volatile __bit RXF1SID4 __attribute__((address(0x7721)));


extern volatile __bit RXF1SID5 __attribute__((address(0x7722)));


extern volatile __bit RXF1SID6 __attribute__((address(0x7723)));


extern volatile __bit RXF1SID7 __attribute__((address(0x7724)));


extern volatile __bit RXF1SID8 __attribute__((address(0x7725)));


extern volatile __bit RXF1SID9 __attribute__((address(0x7726)));


extern volatile __bit RXF2EID0 __attribute__((address(0x7758)));


extern volatile __bit RXF2EID1 __attribute__((address(0x7759)));


extern volatile __bit RXF2EID10 __attribute__((address(0x7752)));


extern volatile __bit RXF2EID11 __attribute__((address(0x7753)));


extern volatile __bit RXF2EID12 __attribute__((address(0x7754)));


extern volatile __bit RXF2EID13 __attribute__((address(0x7755)));


extern volatile __bit RXF2EID14 __attribute__((address(0x7756)));


extern volatile __bit RXF2EID15 __attribute__((address(0x7757)));


extern volatile __bit RXF2EID16 __attribute__((address(0x7748)));


extern volatile __bit RXF2EID17 __attribute__((address(0x7749)));


extern volatile __bit RXF2EID2 __attribute__((address(0x775A)));


extern volatile __bit RXF2EID3 __attribute__((address(0x775B)));


extern volatile __bit RXF2EID4 __attribute__((address(0x775C)));


extern volatile __bit RXF2EID5 __attribute__((address(0x775D)));


extern volatile __bit RXF2EID6 __attribute__((address(0x775E)));


extern volatile __bit RXF2EID7 __attribute__((address(0x775F)));


extern volatile __bit RXF2EID8 __attribute__((address(0x7750)));


extern volatile __bit RXF2EID9 __attribute__((address(0x7751)));


extern volatile __bit RXF2EN __attribute__((address(0x7232)));


extern volatile __bit RXF2EXIDEN __attribute__((address(0x774B)));


extern volatile __bit RXF2SID0 __attribute__((address(0x774D)));


extern volatile __bit RXF2SID1 __attribute__((address(0x774E)));


extern volatile __bit RXF2SID10 __attribute__((address(0x7747)));


extern volatile __bit RXF2SID2 __attribute__((address(0x774F)));


extern volatile __bit RXF2SID3 __attribute__((address(0x7740)));


extern volatile __bit RXF2SID4 __attribute__((address(0x7741)));


extern volatile __bit RXF2SID5 __attribute__((address(0x7742)));


extern volatile __bit RXF2SID6 __attribute__((address(0x7743)));


extern volatile __bit RXF2SID7 __attribute__((address(0x7744)));


extern volatile __bit RXF2SID8 __attribute__((address(0x7745)));


extern volatile __bit RXF2SID9 __attribute__((address(0x7746)));


extern volatile __bit RXF3EID0 __attribute__((address(0x7778)));


extern volatile __bit RXF3EID1 __attribute__((address(0x7779)));


extern volatile __bit RXF3EID10 __attribute__((address(0x7772)));


extern volatile __bit RXF3EID11 __attribute__((address(0x7773)));


extern volatile __bit RXF3EID12 __attribute__((address(0x7774)));


extern volatile __bit RXF3EID13 __attribute__((address(0x7775)));


extern volatile __bit RXF3EID14 __attribute__((address(0x7776)));


extern volatile __bit RXF3EID15 __attribute__((address(0x7777)));


extern volatile __bit RXF3EID16 __attribute__((address(0x7768)));


extern volatile __bit RXF3EID17 __attribute__((address(0x7769)));


extern volatile __bit RXF3EID2 __attribute__((address(0x777A)));


extern volatile __bit RXF3EID3 __attribute__((address(0x777B)));


extern volatile __bit RXF3EID4 __attribute__((address(0x777C)));


extern volatile __bit RXF3EID5 __attribute__((address(0x777D)));


extern volatile __bit RXF3EID6 __attribute__((address(0x777E)));


extern volatile __bit RXF3EID7 __attribute__((address(0x777F)));


extern volatile __bit RXF3EID8 __attribute__((address(0x7770)));


extern volatile __bit RXF3EID9 __attribute__((address(0x7771)));


extern volatile __bit RXF3EN __attribute__((address(0x7233)));


extern volatile __bit RXF3EXIDEN __attribute__((address(0x776B)));


extern volatile __bit RXF3SID0 __attribute__((address(0x776D)));


extern volatile __bit RXF3SID1 __attribute__((address(0x776E)));


extern volatile __bit RXF3SID10 __attribute__((address(0x7767)));


extern volatile __bit RXF3SID2 __attribute__((address(0x776F)));


extern volatile __bit RXF3SID3 __attribute__((address(0x7760)));


extern volatile __bit RXF3SID4 __attribute__((address(0x7761)));


extern volatile __bit RXF3SID5 __attribute__((address(0x7762)));


extern volatile __bit RXF3SID6 __attribute__((address(0x7763)));


extern volatile __bit RXF3SID7 __attribute__((address(0x7764)));


extern volatile __bit RXF3SID8 __attribute__((address(0x7765)));


extern volatile __bit RXF3SID9 __attribute__((address(0x7766)));


extern volatile __bit RXF4EID0 __attribute__((address(0x7798)));


extern volatile __bit RXF4EID1 __attribute__((address(0x7799)));


extern volatile __bit RXF4EID10 __attribute__((address(0x7792)));


extern volatile __bit RXF4EID11 __attribute__((address(0x7793)));


extern volatile __bit RXF4EID12 __attribute__((address(0x7794)));


extern volatile __bit RXF4EID13 __attribute__((address(0x7795)));


extern volatile __bit RXF4EID14 __attribute__((address(0x7796)));


extern volatile __bit RXF4EID15 __attribute__((address(0x7797)));


extern volatile __bit RXF4EID16 __attribute__((address(0x7788)));


extern volatile __bit RXF4EID17 __attribute__((address(0x7789)));


extern volatile __bit RXF4EID2 __attribute__((address(0x779A)));


extern volatile __bit RXF4EID3 __attribute__((address(0x779B)));


extern volatile __bit RXF4EID4 __attribute__((address(0x779C)));


extern volatile __bit RXF4EID5 __attribute__((address(0x779D)));


extern volatile __bit RXF4EID6 __attribute__((address(0x779E)));


extern volatile __bit RXF4EID7 __attribute__((address(0x779F)));


extern volatile __bit RXF4EID8 __attribute__((address(0x7790)));


extern volatile __bit RXF4EID9 __attribute__((address(0x7791)));


extern volatile __bit RXF4EN __attribute__((address(0x7234)));


extern volatile __bit RXF4EXIDEN __attribute__((address(0x778B)));


extern volatile __bit RXF4SID0 __attribute__((address(0x778D)));


extern volatile __bit RXF4SID1 __attribute__((address(0x778E)));


extern volatile __bit RXF4SID10 __attribute__((address(0x7787)));


extern volatile __bit RXF4SID2 __attribute__((address(0x778F)));


extern volatile __bit RXF4SID3 __attribute__((address(0x7780)));


extern volatile __bit RXF4SID4 __attribute__((address(0x7781)));


extern volatile __bit RXF4SID5 __attribute__((address(0x7782)));


extern volatile __bit RXF4SID6 __attribute__((address(0x7783)));


extern volatile __bit RXF4SID7 __attribute__((address(0x7784)));


extern volatile __bit RXF4SID8 __attribute__((address(0x7785)));


extern volatile __bit RXF4SID9 __attribute__((address(0x7786)));


extern volatile __bit RXF5EID0 __attribute__((address(0x77B8)));


extern volatile __bit RXF5EID1 __attribute__((address(0x77B9)));


extern volatile __bit RXF5EID10 __attribute__((address(0x77B2)));


extern volatile __bit RXF5EID11 __attribute__((address(0x77B3)));


extern volatile __bit RXF5EID12 __attribute__((address(0x77B4)));


extern volatile __bit RXF5EID13 __attribute__((address(0x77B5)));


extern volatile __bit RXF5EID14 __attribute__((address(0x77B6)));


extern volatile __bit RXF5EID15 __attribute__((address(0x77B7)));


extern volatile __bit RXF5EID16 __attribute__((address(0x77A8)));


extern volatile __bit RXF5EID17 __attribute__((address(0x77A9)));


extern volatile __bit RXF5EID2 __attribute__((address(0x77BA)));


extern volatile __bit RXF5EID3 __attribute__((address(0x77BB)));


extern volatile __bit RXF5EID4 __attribute__((address(0x77BC)));


extern volatile __bit RXF5EID5 __attribute__((address(0x77BD)));


extern volatile __bit RXF5EID6 __attribute__((address(0x77BE)));


extern volatile __bit RXF5EID7 __attribute__((address(0x77BF)));


extern volatile __bit RXF5EID8 __attribute__((address(0x77B0)));


extern volatile __bit RXF5EID9 __attribute__((address(0x77B1)));


extern volatile __bit RXF5EN __attribute__((address(0x7235)));


extern volatile __bit RXF5EXIDEN __attribute__((address(0x77AB)));


extern volatile __bit RXF5SID0 __attribute__((address(0x77AD)));


extern volatile __bit RXF5SID1 __attribute__((address(0x77AE)));


extern volatile __bit RXF5SID10 __attribute__((address(0x77A7)));


extern volatile __bit RXF5SID2 __attribute__((address(0x77AF)));


extern volatile __bit RXF5SID3 __attribute__((address(0x77A0)));


extern volatile __bit RXF5SID4 __attribute__((address(0x77A1)));


extern volatile __bit RXF5SID5 __attribute__((address(0x77A2)));


extern volatile __bit RXF5SID6 __attribute__((address(0x77A3)));


extern volatile __bit RXF5SID7 __attribute__((address(0x77A4)));


extern volatile __bit RXF5SID8 __attribute__((address(0x77A5)));


extern volatile __bit RXF5SID9 __attribute__((address(0x77A6)));


extern volatile __bit RXF6EID0 __attribute__((address(0x7258)));


extern volatile __bit RXF6EID1 __attribute__((address(0x7259)));


extern volatile __bit RXF6EID10 __attribute__((address(0x7252)));


extern volatile __bit RXF6EID11 __attribute__((address(0x7253)));


extern volatile __bit RXF6EID12 __attribute__((address(0x7254)));


extern volatile __bit RXF6EID13 __attribute__((address(0x7255)));


extern volatile __bit RXF6EID14 __attribute__((address(0x7256)));


extern volatile __bit RXF6EID15 __attribute__((address(0x7257)));


extern volatile __bit RXF6EID16 __attribute__((address(0x7248)));


extern volatile __bit RXF6EID17 __attribute__((address(0x7249)));


extern volatile __bit RXF6EID2 __attribute__((address(0x725A)));


extern volatile __bit RXF6EID3 __attribute__((address(0x725B)));


extern volatile __bit RXF6EID4 __attribute__((address(0x725C)));


extern volatile __bit RXF6EID5 __attribute__((address(0x725D)));


extern volatile __bit RXF6EID6 __attribute__((address(0x725E)));


extern volatile __bit RXF6EID7 __attribute__((address(0x725F)));


extern volatile __bit RXF6EID8 __attribute__((address(0x7250)));


extern volatile __bit RXF6EID9 __attribute__((address(0x7251)));


extern volatile __bit RXF6EN __attribute__((address(0x7236)));


extern volatile __bit RXF6EXIDEN __attribute__((address(0x724B)));


extern volatile __bit RXF6SID0 __attribute__((address(0x724D)));


extern volatile __bit RXF6SID1 __attribute__((address(0x724E)));


extern volatile __bit RXF6SID10 __attribute__((address(0x7247)));


extern volatile __bit RXF6SID2 __attribute__((address(0x724F)));


extern volatile __bit RXF6SID3 __attribute__((address(0x7240)));


extern volatile __bit RXF6SID4 __attribute__((address(0x7241)));


extern volatile __bit RXF6SID5 __attribute__((address(0x7242)));


extern volatile __bit RXF6SID6 __attribute__((address(0x7243)));


extern volatile __bit RXF6SID7 __attribute__((address(0x7244)));


extern volatile __bit RXF6SID8 __attribute__((address(0x7245)));


extern volatile __bit RXF6SID9 __attribute__((address(0x7246)));


extern volatile __bit RXF7EID0 __attribute__((address(0x7278)));


extern volatile __bit RXF7EID1 __attribute__((address(0x7279)));


extern volatile __bit RXF7EID10 __attribute__((address(0x7272)));


extern volatile __bit RXF7EID11 __attribute__((address(0x7273)));


extern volatile __bit RXF7EID12 __attribute__((address(0x7274)));


extern volatile __bit RXF7EID13 __attribute__((address(0x7275)));


extern volatile __bit RXF7EID14 __attribute__((address(0x7276)));


extern volatile __bit RXF7EID15 __attribute__((address(0x7277)));


extern volatile __bit RXF7EID16 __attribute__((address(0x7268)));


extern volatile __bit RXF7EID17 __attribute__((address(0x7269)));


extern volatile __bit RXF7EID2 __attribute__((address(0x727A)));


extern volatile __bit RXF7EID3 __attribute__((address(0x727B)));


extern volatile __bit RXF7EID4 __attribute__((address(0x727C)));


extern volatile __bit RXF7EID5 __attribute__((address(0x727D)));


extern volatile __bit RXF7EID6 __attribute__((address(0x727E)));


extern volatile __bit RXF7EID7 __attribute__((address(0x727F)));


extern volatile __bit RXF7EID8 __attribute__((address(0x7270)));


extern volatile __bit RXF7EID9 __attribute__((address(0x7271)));


extern volatile __bit RXF7EN __attribute__((address(0x7237)));


extern volatile __bit RXF7EXIDEN __attribute__((address(0x726B)));


extern volatile __bit RXF7SID0 __attribute__((address(0x726D)));


extern volatile __bit RXF7SID1 __attribute__((address(0x726E)));


extern volatile __bit RXF7SID10 __attribute__((address(0x7267)));


extern volatile __bit RXF7SID2 __attribute__((address(0x726F)));


extern volatile __bit RXF7SID3 __attribute__((address(0x7260)));


extern volatile __bit RXF7SID4 __attribute__((address(0x7261)));


extern volatile __bit RXF7SID5 __attribute__((address(0x7262)));


extern volatile __bit RXF7SID6 __attribute__((address(0x7263)));


extern volatile __bit RXF7SID7 __attribute__((address(0x7264)));


extern volatile __bit RXF7SID8 __attribute__((address(0x7265)));


extern volatile __bit RXF7SID9 __attribute__((address(0x7266)));


extern volatile __bit RXF8EID0 __attribute__((address(0x7298)));


extern volatile __bit RXF8EID1 __attribute__((address(0x7299)));


extern volatile __bit RXF8EID10 __attribute__((address(0x7292)));


extern volatile __bit RXF8EID11 __attribute__((address(0x7293)));


extern volatile __bit RXF8EID12 __attribute__((address(0x7294)));


extern volatile __bit RXF8EID13 __attribute__((address(0x7295)));


extern volatile __bit RXF8EID14 __attribute__((address(0x7296)));


extern volatile __bit RXF8EID15 __attribute__((address(0x7297)));


extern volatile __bit RXF8EID16 __attribute__((address(0x7288)));


extern volatile __bit RXF8EID17 __attribute__((address(0x7289)));


extern volatile __bit RXF8EID2 __attribute__((address(0x729A)));


extern volatile __bit RXF8EID3 __attribute__((address(0x729B)));


extern volatile __bit RXF8EID4 __attribute__((address(0x729C)));


extern volatile __bit RXF8EID5 __attribute__((address(0x729D)));


extern volatile __bit RXF8EID6 __attribute__((address(0x729E)));


extern volatile __bit RXF8EID7 __attribute__((address(0x729F)));


extern volatile __bit RXF8EID8 __attribute__((address(0x7290)));


extern volatile __bit RXF8EID9 __attribute__((address(0x7291)));


extern volatile __bit RXF8EN __attribute__((address(0x7238)));


extern volatile __bit RXF8EXIDEN __attribute__((address(0x728B)));


extern volatile __bit RXF8SID0 __attribute__((address(0x728D)));


extern volatile __bit RXF8SID1 __attribute__((address(0x728E)));


extern volatile __bit RXF8SID10 __attribute__((address(0x7287)));


extern volatile __bit RXF8SID2 __attribute__((address(0x728F)));


extern volatile __bit RXF8SID3 __attribute__((address(0x7280)));


extern volatile __bit RXF8SID4 __attribute__((address(0x7281)));


extern volatile __bit RXF8SID5 __attribute__((address(0x7282)));


extern volatile __bit RXF8SID6 __attribute__((address(0x7283)));


extern volatile __bit RXF8SID7 __attribute__((address(0x7284)));


extern volatile __bit RXF8SID8 __attribute__((address(0x7285)));


extern volatile __bit RXF8SID9 __attribute__((address(0x7286)));


extern volatile __bit RXF9EID0 __attribute__((address(0x72B8)));


extern volatile __bit RXF9EID1 __attribute__((address(0x72B9)));


extern volatile __bit RXF9EID10 __attribute__((address(0x72B2)));


extern volatile __bit RXF9EID11 __attribute__((address(0x72B3)));


extern volatile __bit RXF9EID12 __attribute__((address(0x72B4)));


extern volatile __bit RXF9EID13 __attribute__((address(0x72B5)));


extern volatile __bit RXF9EID14 __attribute__((address(0x72B6)));


extern volatile __bit RXF9EID15 __attribute__((address(0x72B7)));


extern volatile __bit RXF9EID16 __attribute__((address(0x72A8)));


extern volatile __bit RXF9EID17 __attribute__((address(0x72A9)));


extern volatile __bit RXF9EID2 __attribute__((address(0x72BA)));


extern volatile __bit RXF9EID3 __attribute__((address(0x72BB)));


extern volatile __bit RXF9EID4 __attribute__((address(0x72BC)));


extern volatile __bit RXF9EID5 __attribute__((address(0x72BD)));


extern volatile __bit RXF9EID6 __attribute__((address(0x72BE)));


extern volatile __bit RXF9EID7 __attribute__((address(0x72BF)));


extern volatile __bit RXF9EID8 __attribute__((address(0x72B0)));


extern volatile __bit RXF9EID9 __attribute__((address(0x72B1)));


extern volatile __bit RXF9EN __attribute__((address(0x7239)));


extern volatile __bit RXF9EXIDEN __attribute__((address(0x72AB)));


extern volatile __bit RXF9SID0 __attribute__((address(0x72AD)));


extern volatile __bit RXF9SID1 __attribute__((address(0x72AE)));


extern volatile __bit RXF9SID10 __attribute__((address(0x72A7)));


extern volatile __bit RXF9SID2 __attribute__((address(0x72AF)));


extern volatile __bit RXF9SID3 __attribute__((address(0x72A0)));


extern volatile __bit RXF9SID4 __attribute__((address(0x72A1)));


extern volatile __bit RXF9SID5 __attribute__((address(0x72A2)));


extern volatile __bit RXF9SID6 __attribute__((address(0x72A3)));


extern volatile __bit RXF9SID7 __attribute__((address(0x72A4)));


extern volatile __bit RXF9SID8 __attribute__((address(0x72A5)));


extern volatile __bit RXF9SID9 __attribute__((address(0x72A6)));


extern volatile __bit RXM0EID0 __attribute__((address(0x77D8)));


extern volatile __bit RXM0EID1 __attribute__((address(0x77D9)));


extern volatile __bit RXM0EID10 __attribute__((address(0x77D2)));


extern volatile __bit RXM0EID11 __attribute__((address(0x77D3)));


extern volatile __bit RXM0EID12 __attribute__((address(0x77D4)));


extern volatile __bit RXM0EID13 __attribute__((address(0x77D5)));


extern volatile __bit RXM0EID14 __attribute__((address(0x77D6)));


extern volatile __bit RXM0EID15 __attribute__((address(0x77D7)));


extern volatile __bit RXM0EID16 __attribute__((address(0x77C8)));


extern volatile __bit RXM0EID17 __attribute__((address(0x77C9)));


extern volatile __bit RXM0EID2 __attribute__((address(0x77DA)));


extern volatile __bit RXM0EID3 __attribute__((address(0x77DB)));


extern volatile __bit RXM0EID4 __attribute__((address(0x77DC)));


extern volatile __bit RXM0EID5 __attribute__((address(0x77DD)));


extern volatile __bit RXM0EID6 __attribute__((address(0x77DE)));


extern volatile __bit RXM0EID7 __attribute__((address(0x77DF)));


extern volatile __bit RXM0EID8 __attribute__((address(0x77D0)));


extern volatile __bit RXM0EID9 __attribute__((address(0x77D1)));


extern volatile __bit RXM0EXIDM __attribute__((address(0x77CB)));


extern volatile __bit RXM0SID0 __attribute__((address(0x77CD)));


extern volatile __bit RXM0SID1 __attribute__((address(0x77CE)));


extern volatile __bit RXM0SID10 __attribute__((address(0x77C7)));


extern volatile __bit RXM0SID2 __attribute__((address(0x77CF)));


extern volatile __bit RXM0SID3 __attribute__((address(0x77C0)));


extern volatile __bit RXM0SID4 __attribute__((address(0x77C1)));


extern volatile __bit RXM0SID5 __attribute__((address(0x77C2)));


extern volatile __bit RXM0SID6 __attribute__((address(0x77C3)));


extern volatile __bit RXM0SID7 __attribute__((address(0x77C4)));


extern volatile __bit RXM0SID8 __attribute__((address(0x77C5)));


extern volatile __bit RXM0SID9 __attribute__((address(0x77C6)));


extern volatile __bit RXM1EID0 __attribute__((address(0x77F8)));


extern volatile __bit RXM1EID1 __attribute__((address(0x77F9)));


extern volatile __bit RXM1EID10 __attribute__((address(0x77F2)));


extern volatile __bit RXM1EID11 __attribute__((address(0x77F3)));


extern volatile __bit RXM1EID12 __attribute__((address(0x77F4)));


extern volatile __bit RXM1EID13 __attribute__((address(0x77F5)));


extern volatile __bit RXM1EID14 __attribute__((address(0x77F6)));


extern volatile __bit RXM1EID15 __attribute__((address(0x77F7)));


extern volatile __bit RXM1EID16 __attribute__((address(0x77E8)));


extern volatile __bit RXM1EID17 __attribute__((address(0x77E9)));


extern volatile __bit RXM1EID2 __attribute__((address(0x77FA)));


extern volatile __bit RXM1EID3 __attribute__((address(0x77FB)));


extern volatile __bit RXM1EID4 __attribute__((address(0x77FC)));


extern volatile __bit RXM1EID5 __attribute__((address(0x77FD)));


extern volatile __bit RXM1EID6 __attribute__((address(0x77FE)));


extern volatile __bit RXM1EID7 __attribute__((address(0x77FF)));


extern volatile __bit RXM1EID8 __attribute__((address(0x77F0)));


extern volatile __bit RXM1EID9 __attribute__((address(0x77F1)));


extern volatile __bit RXM1EXIDEN __attribute__((address(0x77EB)));


extern volatile __bit RXM1SID0 __attribute__((address(0x77ED)));


extern volatile __bit RXM1SID1 __attribute__((address(0x77EE)));


extern volatile __bit RXM1SID10 __attribute__((address(0x77E7)));


extern volatile __bit RXM1SID2 __attribute__((address(0x77EF)));


extern volatile __bit RXM1SID3 __attribute__((address(0x77E0)));


extern volatile __bit RXM1SID4 __attribute__((address(0x77E1)));


extern volatile __bit RXM1SID5 __attribute__((address(0x77E2)));


extern volatile __bit RXM1SID6 __attribute__((address(0x77E3)));


extern volatile __bit RXM1SID7 __attribute__((address(0x77E4)));


extern volatile __bit RXM1SID8 __attribute__((address(0x77E5)));


extern volatile __bit RXM1SID9 __attribute__((address(0x77E6)));


extern volatile __bit RXWARN __attribute__((address(0x7B89)));


extern volatile __bit R_NOT_W __attribute__((address(0x7E3A)));


extern volatile __bit R_W __attribute__((address(0x7E3A)));


extern volatile __bit R_nW __attribute__((address(0x7E3A)));


extern volatile __bit SAM __attribute__((address(0x7226)));


extern volatile __bit SBOREN __attribute__((address(0x7E86)));


extern volatile __bit SCKP __attribute__((address(0x7D3C)));


extern volatile __bit SCKP1 __attribute__((address(0x7D3C)));


extern volatile __bit SCKP2 __attribute__((address(0x7DCC)));


extern volatile __bit SCS0 __attribute__((address(0x7E98)));


extern volatile __bit SCS1 __attribute__((address(0x7E99)));


extern volatile __bit SEG1PH0 __attribute__((address(0x7223)));


extern volatile __bit SEG1PH1 __attribute__((address(0x7224)));


extern volatile __bit SEG1PH2 __attribute__((address(0x7225)));


extern volatile __bit SEG2PH0 __attribute__((address(0x7228)));


extern volatile __bit SEG2PH1 __attribute__((address(0x7229)));


extern volatile __bit SEG2PH2 __attribute__((address(0x722A)));


extern volatile __bit SEG2PHT __attribute__((address(0x7227)));


extern volatile __bit SEG2PHTS __attribute__((address(0x7227)));


extern volatile __bit SEN __attribute__((address(0x7E28)));


extern volatile __bit SENDB1 __attribute__((address(0x7D63)));


extern volatile __bit SENDB2 __attribute__((address(0x7DD3)));


extern volatile __bit SJW0 __attribute__((address(0x721E)));


extern volatile __bit SJW1 __attribute__((address(0x721F)));


extern volatile __bit SLRA __attribute__((address(0x7C80)));


extern volatile __bit SLRB __attribute__((address(0x7C81)));


extern volatile __bit SLRC __attribute__((address(0x7C82)));


extern volatile __bit SMP __attribute__((address(0x7E3F)));


extern volatile __bit SOSCDRV __attribute__((address(0x7E94)));


extern volatile __bit SOSCEN3 __attribute__((address(0x7D8B)));


extern volatile __bit SOSCGO __attribute__((address(0x7E93)));


extern volatile __bit SOSCRUN __attribute__((address(0x7E96)));


extern volatile __bit SP0 __attribute__((address(0x7FE0)));


extern volatile __bit SP1 __attribute__((address(0x7FE1)));


extern volatile __bit SP2 __attribute__((address(0x7FE2)));


extern volatile __bit SP3 __attribute__((address(0x7FE3)));


extern volatile __bit SP4 __attribute__((address(0x7FE4)));


extern volatile __bit SPEN1 __attribute__((address(0x7D5F)));


extern volatile __bit SPEN2 __attribute__((address(0x7D37)));


extern volatile __bit SPI1MD __attribute__((address(0x7AC9)));


extern volatile __bit SPI2MD __attribute__((address(0x7ACA)));


extern volatile __bit SREN1 __attribute__((address(0x7D5D)));


extern volatile __bit SREN2 __attribute__((address(0x7D35)));


extern volatile __bit SRENA __attribute__((address(0x7D5D)));


extern volatile __bit SRETEN __attribute__((address(0x7E8C)));


extern volatile __bit SSPEN __attribute__((address(0x7E35)));


extern volatile __bit SSPIE __attribute__((address(0x7CEB)));


extern volatile __bit SSPIF __attribute__((address(0x7CF3)));


extern volatile __bit SSPIP __attribute__((address(0x7CFB)));


extern volatile __bit SSPM0 __attribute__((address(0x7E30)));


extern volatile __bit SSPM1 __attribute__((address(0x7E31)));


extern volatile __bit SSPM2 __attribute__((address(0x7E32)));


extern volatile __bit SSPM3 __attribute__((address(0x7E33)));


extern volatile __bit SSPMD __attribute__((address(0x7AC8)));


extern volatile __bit SSPOD __attribute__((address(0x7C8F)));


extern volatile __bit SSPOV __attribute__((address(0x7E36)));


extern volatile __bit START __attribute__((address(0x7E3B)));


extern volatile __bit STKFUL __attribute__((address(0x7FE7)));


extern volatile __bit STKOVF __attribute__((address(0x7FE7)));


extern volatile __bit STKUNF __attribute__((address(0x7FE6)));


extern volatile __bit STOP __attribute__((address(0x7E3C)));


extern volatile __bit STRA __attribute__((address(0x7CE0)));


extern volatile __bit STRB __attribute__((address(0x7CE1)));


extern volatile __bit STRC __attribute__((address(0x7CE2)));


extern volatile __bit STRD __attribute__((address(0x7CE3)));


extern volatile __bit STRSYNC __attribute__((address(0x7CE4)));


extern volatile __bit SWDTE __attribute__((address(0x7E88)));


extern volatile __bit SWDTEN __attribute__((address(0x7E88)));


extern volatile __bit SYNC1 __attribute__((address(0x7D64)));


extern volatile __bit SYNC2 __attribute__((address(0x7DD4)));


extern volatile __bit T08BIT __attribute__((address(0x7EAE)));


extern volatile __bit T0CS __attribute__((address(0x7EAD)));


extern volatile __bit T0IE __attribute__((address(0x7F95)));


extern volatile __bit T0IF __attribute__((address(0x7F92)));


extern volatile __bit T0IP __attribute__((address(0x7F8A)));


extern volatile __bit T0PS0 __attribute__((address(0x7EA8)));


extern volatile __bit T0PS1 __attribute__((address(0x7EA9)));


extern volatile __bit T0PS2 __attribute__((address(0x7EAA)));


extern volatile __bit T0SE __attribute__((address(0x7EAC)));


extern volatile __bit T1CKPS0 __attribute__((address(0x7E6C)));


extern volatile __bit T1CKPS1 __attribute__((address(0x7E6D)));


extern volatile __bit T1GGO __attribute__((address(0x7D53)));


extern volatile __bit T1GGO_NOT_T1DONE __attribute__((address(0x7D53)));


extern volatile __bit T1GGO_nT1DONE __attribute__((address(0x7D53)));


extern volatile __bit T1GPOL __attribute__((address(0x7D56)));


extern volatile __bit T1GSPM __attribute__((address(0x7D54)));


extern volatile __bit T1GSS0 __attribute__((address(0x7D50)));


extern volatile __bit T1GSS1 __attribute__((address(0x7D51)));


extern volatile __bit T1GTM __attribute__((address(0x7D55)));


extern volatile __bit T1GVAL __attribute__((address(0x7D52)));


extern volatile __bit T1OSCEN __attribute__((address(0x7E6B)));


extern volatile __bit T1RD16 __attribute__((address(0x7E6F)));


extern volatile __bit T2CKPS0 __attribute__((address(0x7E50)));


extern volatile __bit T2CKPS1 __attribute__((address(0x7E51)));


extern volatile __bit T2OUTPS0 __attribute__((address(0x7E53)));


extern volatile __bit T2OUTPS1 __attribute__((address(0x7E54)));


extern volatile __bit T2OUTPS2 __attribute__((address(0x7E55)));


extern volatile __bit T2OUTPS3 __attribute__((address(0x7E56)));


extern volatile __bit T3CKPS0 __attribute__((address(0x7D8C)));


extern volatile __bit T3CKPS1 __attribute__((address(0x7D8D)));


extern volatile __bit T3GGO __attribute__((address(0x7D83)));


extern volatile __bit T3GGO_NOT_T3DONE __attribute__((address(0x7D83)));


extern volatile __bit T3GGO_nT3DONE __attribute__((address(0x7D83)));


extern volatile __bit T3GPOL __attribute__((address(0x7D86)));


extern volatile __bit T3GSPM __attribute__((address(0x7D84)));


extern volatile __bit T3GSS0 __attribute__((address(0x7D80)));


extern volatile __bit T3GSS1 __attribute__((address(0x7D81)));


extern volatile __bit T3GTM __attribute__((address(0x7D85)));


extern volatile __bit T3GVAL __attribute__((address(0x7D82)));


extern volatile __bit T3RD16 __attribute__((address(0x7D8F)));


extern volatile __bit T4CKPS0 __attribute__((address(0x7C40)));


extern volatile __bit T4CKPS1 __attribute__((address(0x7C41)));


extern volatile __bit T4OUTPS0 __attribute__((address(0x7C43)));


extern volatile __bit T4OUTPS1 __attribute__((address(0x7C44)));


extern volatile __bit T4OUTPS2 __attribute__((address(0x7C45)));


extern volatile __bit T4OUTPS3 __attribute__((address(0x7C46)));


extern volatile __bit TEC0 __attribute__((address(0x7210)));


extern volatile __bit TEC1 __attribute__((address(0x7211)));


extern volatile __bit TEC2 __attribute__((address(0x7212)));


extern volatile __bit TEC3 __attribute__((address(0x7213)));


extern volatile __bit TEC4 __attribute__((address(0x7214)));


extern volatile __bit TEC5 __attribute__((address(0x7215)));


extern volatile __bit TEC6 __attribute__((address(0x7216)));


extern volatile __bit TEC7 __attribute__((address(0x7217)));


extern volatile __bit TGEN __attribute__((address(0x7AAC)));


extern volatile __bit TMR0IE __attribute__((address(0x7F95)));


extern volatile __bit TMR0IF __attribute__((address(0x7F92)));


extern volatile __bit TMR0IP __attribute__((address(0x7F8A)));


extern volatile __bit TMR0MD __attribute__((address(0x7AC0)));


extern volatile __bit TMR0ON __attribute__((address(0x7EAF)));


extern volatile __bit TMR1CS0 __attribute__((address(0x7E6E)));


extern volatile __bit TMR1CS1 __attribute__((address(0x7E6F)));


extern volatile __bit TMR1GE __attribute__((address(0x7D57)));


extern volatile __bit TMR1GIE __attribute__((address(0x7CEA)));


extern volatile __bit TMR1GIF __attribute__((address(0x7CF2)));


extern volatile __bit TMR1GIP __attribute__((address(0x7CFA)));


extern volatile __bit TMR1IE __attribute__((address(0x7CE8)));


extern volatile __bit TMR1IF __attribute__((address(0x7CF0)));


extern volatile __bit TMR1IP __attribute__((address(0x7CF8)));


extern volatile __bit TMR1MD __attribute__((address(0x7AC1)));


extern volatile __bit TMR1ON __attribute__((address(0x7E68)));


extern volatile __bit TMR2IE __attribute__((address(0x7CE9)));


extern volatile __bit TMR2IF __attribute__((address(0x7CF1)));


extern volatile __bit TMR2IP __attribute__((address(0x7CF9)));


extern volatile __bit TMR2MD __attribute__((address(0x7AC2)));


extern volatile __bit TMR2ON __attribute__((address(0x7E52)));


extern volatile __bit TMR3CS0 __attribute__((address(0x7D8E)));


extern volatile __bit TMR3CS1 __attribute__((address(0x7D8F)));


extern volatile __bit TMR3GE __attribute__((address(0x7D87)));


extern volatile __bit TMR3GIE __attribute__((address(0x7D00)));


extern volatile __bit TMR3GIF __attribute__((address(0x7D08)));


extern volatile __bit TMR3GIP __attribute__((address(0x7D10)));


extern volatile __bit TMR3IE __attribute__((address(0x7D01)));


extern volatile __bit TMR3IF __attribute__((address(0x7D09)));


extern volatile __bit TMR3IP __attribute__((address(0x7D11)));


extern volatile __bit TMR3MD __attribute__((address(0x7AC3)));


extern volatile __bit TMR3ON __attribute__((address(0x7D88)));


extern volatile __bit TMR4IE __attribute__((address(0x7DB7)));


extern volatile __bit TMR4IF __attribute__((address(0x7DBF)));


extern volatile __bit TMR4IP __attribute__((address(0x7DC7)));


extern volatile __bit TMR4MD __attribute__((address(0x7AC4)));


extern volatile __bit TMR4ON __attribute__((address(0x7C42)));


extern volatile __bit TO __attribute__((address(0x7E83)));


extern volatile __bit TRIGSEL0 __attribute__((address(0x7E0E)));


extern volatile __bit TRIGSEL1 __attribute__((address(0x7E0F)));


extern volatile __bit TRISA0 __attribute__((address(0x7C90)));


extern volatile __bit TRISA1 __attribute__((address(0x7C91)));


extern volatile __bit TRISA2 __attribute__((address(0x7C92)));


extern volatile __bit TRISA3 __attribute__((address(0x7C93)));


extern volatile __bit TRISA5 __attribute__((address(0x7C95)));


extern volatile __bit TRISA6 __attribute__((address(0x7C96)));


extern volatile __bit TRISA7 __attribute__((address(0x7C97)));


extern volatile __bit TRISB0 __attribute__((address(0x7C98)));


extern volatile __bit TRISB1 __attribute__((address(0x7C99)));


extern volatile __bit TRISB2 __attribute__((address(0x7C9A)));


extern volatile __bit TRISB3 __attribute__((address(0x7C9B)));


extern volatile __bit TRISB4 __attribute__((address(0x7C9C)));


extern volatile __bit TRISB5 __attribute__((address(0x7C9D)));


extern volatile __bit TRISB6 __attribute__((address(0x7C9E)));


extern volatile __bit TRISB7 __attribute__((address(0x7C9F)));


extern volatile __bit TRISC0 __attribute__((address(0x7CA0)));


extern volatile __bit TRISC1 __attribute__((address(0x7CA1)));


extern volatile __bit TRISC2 __attribute__((address(0x7CA2)));


extern volatile __bit TRISC3 __attribute__((address(0x7CA3)));


extern volatile __bit TRISC4 __attribute__((address(0x7CA4)));


extern volatile __bit TRISC5 __attribute__((address(0x7CA5)));


extern volatile __bit TRISC6 __attribute__((address(0x7CA6)));


extern volatile __bit TRISC7 __attribute__((address(0x7CA7)));


extern volatile __bit TRMT1 __attribute__((address(0x7D61)));


extern volatile __bit TRMT2 __attribute__((address(0x7DD1)));


extern volatile __bit TUN0 __attribute__((address(0x7CD8)));


extern volatile __bit TUN1 __attribute__((address(0x7CD9)));


extern volatile __bit TUN2 __attribute__((address(0x7CDA)));


extern volatile __bit TUN3 __attribute__((address(0x7CDB)));


extern volatile __bit TUN4 __attribute__((address(0x7CDC)));


extern volatile __bit TUN5 __attribute__((address(0x7CDD)));


extern volatile __bit TX0IE __attribute__((address(0x73FA)));


extern volatile __bit TX0IF __attribute__((address(0x7907)));


extern volatile __bit __attribute__((__deprecated__)) TX1IE __attribute__((address(0x7CEC)));


extern volatile __bit __attribute__((__deprecated__)) TX1IF __attribute__((address(0x7CF4)));


extern volatile __bit TX1IP __attribute__((address(0x7CFC)));


extern volatile __bit TX2EN __attribute__((address(0x7B86)));


extern volatile __bit __attribute__((__deprecated__)) TX2IE __attribute__((address(0x7D1C)));


extern volatile __bit __attribute__((__deprecated__)) TX2IF __attribute__((address(0x7D24)));


extern volatile __bit TX2IP __attribute__((address(0x7D2C)));


extern volatile __bit TX2SRC __attribute__((address(0x7B87)));


extern volatile __bit TX8_9 __attribute__((address(0x7D66)));


extern volatile __bit TX8_92 __attribute__((address(0x7DD6)));


extern volatile __bit TX91 __attribute__((address(0x7D66)));


extern volatile __bit TX92 __attribute__((address(0x7DD6)));


extern volatile __bit TX9D1 __attribute__((address(0x7D60)));


extern volatile __bit TX9D2 __attribute__((address(0x7DD0)));


extern volatile __bit TXB0ABT __attribute__((address(0x7906)));


extern volatile __bit TXB0D00 __attribute__((address(0x7930)));


extern volatile __bit TXB0D01 __attribute__((address(0x7931)));


extern volatile __bit TXB0D02 __attribute__((address(0x7932)));


extern volatile __bit TXB0D03 __attribute__((address(0x7933)));


extern volatile __bit TXB0D04 __attribute__((address(0x7934)));


extern volatile __bit TXB0D05 __attribute__((address(0x7935)));


extern volatile __bit TXB0D06 __attribute__((address(0x7936)));


extern volatile __bit TXB0D07 __attribute__((address(0x7937)));


extern volatile __bit TXB0D10 __attribute__((address(0x7938)));


extern volatile __bit TXB0D11 __attribute__((address(0x7939)));


extern volatile __bit TXB0D12 __attribute__((address(0x793A)));


extern volatile __bit TXB0D13 __attribute__((address(0x793B)));


extern volatile __bit TXB0D14 __attribute__((address(0x793C)));


extern volatile __bit TXB0D15 __attribute__((address(0x793D)));


extern volatile __bit TXB0D16 __attribute__((address(0x793E)));


extern volatile __bit TXB0D17 __attribute__((address(0x793F)));


extern volatile __bit TXB0D20 __attribute__((address(0x7940)));


extern volatile __bit TXB0D21 __attribute__((address(0x7941)));


extern volatile __bit TXB0D22 __attribute__((address(0x7942)));


extern volatile __bit TXB0D23 __attribute__((address(0x7943)));


extern volatile __bit TXB0D24 __attribute__((address(0x7944)));


extern volatile __bit TXB0D25 __attribute__((address(0x7945)));


extern volatile __bit TXB0D26 __attribute__((address(0x7946)));


extern volatile __bit TXB0D27 __attribute__((address(0x7947)));


extern volatile __bit TXB0D30 __attribute__((address(0x7948)));


extern volatile __bit TXB0D31 __attribute__((address(0x7949)));


extern volatile __bit TXB0D32 __attribute__((address(0x794A)));


extern volatile __bit TXB0D33 __attribute__((address(0x794B)));


extern volatile __bit TXB0D34 __attribute__((address(0x794C)));


extern volatile __bit TXB0D35 __attribute__((address(0x794D)));


extern volatile __bit TXB0D36 __attribute__((address(0x794E)));


extern volatile __bit TXB0D37 __attribute__((address(0x794F)));


extern volatile __bit TXB0D40 __attribute__((address(0x7950)));


extern volatile __bit TXB0D41 __attribute__((address(0x7951)));


extern volatile __bit TXB0D42 __attribute__((address(0x7952)));


extern volatile __bit TXB0D43 __attribute__((address(0x7953)));


extern volatile __bit TXB0D44 __attribute__((address(0x7954)));


extern volatile __bit TXB0D45 __attribute__((address(0x7955)));


extern volatile __bit TXB0D46 __attribute__((address(0x7956)));


extern volatile __bit TXB0D47 __attribute__((address(0x7957)));


extern volatile __bit TXB0D50 __attribute__((address(0x7958)));


extern volatile __bit TXB0D51 __attribute__((address(0x7959)));


extern volatile __bit TXB0D52 __attribute__((address(0x795A)));


extern volatile __bit TXB0D53 __attribute__((address(0x795B)));


extern volatile __bit TXB0D54 __attribute__((address(0x795C)));


extern volatile __bit TXB0D55 __attribute__((address(0x795D)));


extern volatile __bit TXB0D56 __attribute__((address(0x795E)));


extern volatile __bit TXB0D57 __attribute__((address(0x795F)));


extern volatile __bit TXB0D60 __attribute__((address(0x7960)));


extern volatile __bit TXB0D61 __attribute__((address(0x7961)));


extern volatile __bit TXB0D62 __attribute__((address(0x7962)));


extern volatile __bit TXB0D63 __attribute__((address(0x7963)));


extern volatile __bit TXB0D64 __attribute__((address(0x7964)));


extern volatile __bit TXB0D65 __attribute__((address(0x7965)));


extern volatile __bit TXB0D66 __attribute__((address(0x7966)));


extern volatile __bit TXB0D67 __attribute__((address(0x7967)));


extern volatile __bit TXB0D70 __attribute__((address(0x7968)));


extern volatile __bit TXB0D71 __attribute__((address(0x7969)));


extern volatile __bit TXB0D72 __attribute__((address(0x796A)));


extern volatile __bit TXB0D73 __attribute__((address(0x796B)));


extern volatile __bit TXB0D74 __attribute__((address(0x796C)));


extern volatile __bit TXB0D75 __attribute__((address(0x796D)));


extern volatile __bit TXB0D76 __attribute__((address(0x796E)));


extern volatile __bit TXB0D77 __attribute__((address(0x796F)));


extern volatile __bit TXB0DLC0 __attribute__((address(0x7928)));


extern volatile __bit TXB0DLC1 __attribute__((address(0x7929)));


extern volatile __bit TXB0DLC2 __attribute__((address(0x792A)));


extern volatile __bit TXB0DLC3 __attribute__((address(0x792B)));


extern volatile __bit TXB0EID0 __attribute__((address(0x7920)));


extern volatile __bit TXB0EID1 __attribute__((address(0x7921)));


extern volatile __bit TXB0EID10 __attribute__((address(0x791A)));


extern volatile __bit TXB0EID11 __attribute__((address(0x791B)));


extern volatile __bit TXB0EID12 __attribute__((address(0x791C)));


extern volatile __bit TXB0EID13 __attribute__((address(0x791D)));


extern volatile __bit TXB0EID14 __attribute__((address(0x791E)));


extern volatile __bit TXB0EID15 __attribute__((address(0x791F)));


extern volatile __bit TXB0EID16 __attribute__((address(0x7910)));


extern volatile __bit TXB0EID17 __attribute__((address(0x7911)));


extern volatile __bit TXB0EID2 __attribute__((address(0x7922)));


extern volatile __bit TXB0EID3 __attribute__((address(0x7923)));


extern volatile __bit TXB0EID4 __attribute__((address(0x7924)));


extern volatile __bit TXB0EID5 __attribute__((address(0x7925)));


extern volatile __bit TXB0EID6 __attribute__((address(0x7926)));


extern volatile __bit TXB0EID7 __attribute__((address(0x7927)));


extern volatile __bit TXB0EID8 __attribute__((address(0x7918)));


extern volatile __bit TXB0EID9 __attribute__((address(0x7919)));


extern volatile __bit TXB0ERR __attribute__((address(0x7904)));


extern volatile __bit TXB0EXIDE __attribute__((address(0x7913)));


extern volatile __bit TXB0IF __attribute__((address(0x7BBA)));


extern volatile __bit TXB0IP __attribute__((address(0x7BC2)));


extern volatile __bit TXB0LARB __attribute__((address(0x7905)));


extern volatile __bit TXB0PRI0 __attribute__((address(0x7900)));


extern volatile __bit TXB0PRI1 __attribute__((address(0x7901)));


extern volatile __bit TXB0REQ __attribute__((address(0x7903)));


extern volatile __bit TXB0RTR __attribute__((address(0x792E)));


extern volatile __bit TXB0SID0 __attribute__((address(0x7915)));


extern volatile __bit TXB0SID1 __attribute__((address(0x7916)));


extern volatile __bit TXB0SID10 __attribute__((address(0x790F)));


extern volatile __bit TXB0SID2 __attribute__((address(0x7917)));


extern volatile __bit TXB0SID3 __attribute__((address(0x7908)));


extern volatile __bit TXB0SID4 __attribute__((address(0x7909)));


extern volatile __bit TXB0SID5 __attribute__((address(0x790A)));


extern volatile __bit TXB0SID6 __attribute__((address(0x790B)));


extern volatile __bit TXB0SID7 __attribute__((address(0x790C)));


extern volatile __bit TXB0SID8 __attribute__((address(0x790D)));


extern volatile __bit TXB0SID9 __attribute__((address(0x790E)));


extern volatile __bit TXB1ABT __attribute__((address(0x7886)));


extern volatile __bit TXB1D00 __attribute__((address(0x78B0)));


extern volatile __bit TXB1D01 __attribute__((address(0x78B1)));


extern volatile __bit TXB1D02 __attribute__((address(0x78B2)));


extern volatile __bit TXB1D03 __attribute__((address(0x78B3)));


extern volatile __bit TXB1D04 __attribute__((address(0x78B4)));


extern volatile __bit TXB1D05 __attribute__((address(0x78B5)));


extern volatile __bit TXB1D06 __attribute__((address(0x78B6)));


extern volatile __bit TXB1D07 __attribute__((address(0x78B7)));


extern volatile __bit TXB1D10 __attribute__((address(0x78B8)));


extern volatile __bit TXB1D11 __attribute__((address(0x78B9)));


extern volatile __bit TXB1D12 __attribute__((address(0x78BA)));


extern volatile __bit TXB1D13 __attribute__((address(0x78BB)));


extern volatile __bit TXB1D14 __attribute__((address(0x78BC)));


extern volatile __bit TXB1D15 __attribute__((address(0x78BD)));


extern volatile __bit TXB1D16 __attribute__((address(0x78BE)));


extern volatile __bit TXB1D17 __attribute__((address(0x78BF)));


extern volatile __bit TXB1D20 __attribute__((address(0x78C0)));


extern volatile __bit TXB1D21 __attribute__((address(0x78C1)));


extern volatile __bit TXB1D22 __attribute__((address(0x78C2)));


extern volatile __bit TXB1D23 __attribute__((address(0x78C3)));


extern volatile __bit TXB1D24 __attribute__((address(0x78C4)));


extern volatile __bit TXB1D25 __attribute__((address(0x78C5)));


extern volatile __bit TXB1D26 __attribute__((address(0x78C6)));


extern volatile __bit TXB1D27 __attribute__((address(0x78C7)));


extern volatile __bit TXB1D30 __attribute__((address(0x78C8)));


extern volatile __bit TXB1D31 __attribute__((address(0x78C9)));


extern volatile __bit TXB1D32 __attribute__((address(0x78CA)));


extern volatile __bit TXB1D33 __attribute__((address(0x78CB)));


extern volatile __bit TXB1D34 __attribute__((address(0x78CC)));


extern volatile __bit TXB1D35 __attribute__((address(0x78CD)));


extern volatile __bit TXB1D36 __attribute__((address(0x78CE)));


extern volatile __bit TXB1D37 __attribute__((address(0x78CF)));


extern volatile __bit TXB1D40 __attribute__((address(0x78D0)));


extern volatile __bit TXB1D41 __attribute__((address(0x78D1)));


extern volatile __bit TXB1D42 __attribute__((address(0x78D2)));


extern volatile __bit TXB1D43 __attribute__((address(0x78D3)));


extern volatile __bit TXB1D44 __attribute__((address(0x78D4)));


extern volatile __bit TXB1D45 __attribute__((address(0x78D5)));


extern volatile __bit TXB1D46 __attribute__((address(0x78D6)));


extern volatile __bit TXB1D47 __attribute__((address(0x78D7)));


extern volatile __bit TXB1D50 __attribute__((address(0x78D8)));


extern volatile __bit TXB1D51 __attribute__((address(0x78D9)));


extern volatile __bit TXB1D52 __attribute__((address(0x78DA)));


extern volatile __bit TXB1D53 __attribute__((address(0x78DB)));


extern volatile __bit TXB1D54 __attribute__((address(0x78DC)));


extern volatile __bit TXB1D55 __attribute__((address(0x78DD)));


extern volatile __bit TXB1D56 __attribute__((address(0x78DE)));


extern volatile __bit TXB1D57 __attribute__((address(0x78DF)));


extern volatile __bit TXB1D60 __attribute__((address(0x78E0)));


extern volatile __bit TXB1D61 __attribute__((address(0x78E1)));


extern volatile __bit TXB1D62 __attribute__((address(0x78E2)));


extern volatile __bit TXB1D63 __attribute__((address(0x78E3)));


extern volatile __bit TXB1D64 __attribute__((address(0x78E4)));


extern volatile __bit TXB1D65 __attribute__((address(0x78E5)));


extern volatile __bit TXB1D66 __attribute__((address(0x78E6)));


extern volatile __bit TXB1D67 __attribute__((address(0x78E7)));


extern volatile __bit TXB1D70 __attribute__((address(0x78E8)));


extern volatile __bit TXB1D71 __attribute__((address(0x78E9)));


extern volatile __bit TXB1D72 __attribute__((address(0x78EA)));


extern volatile __bit TXB1D73 __attribute__((address(0x78EB)));


extern volatile __bit TXB1D74 __attribute__((address(0x78EC)));


extern volatile __bit TXB1D75 __attribute__((address(0x78ED)));


extern volatile __bit TXB1D76 __attribute__((address(0x78EE)));


extern volatile __bit TXB1D77 __attribute__((address(0x78EF)));


extern volatile __bit TXB1DLC0 __attribute__((address(0x78A8)));


extern volatile __bit TXB1DLC1 __attribute__((address(0x78A9)));


extern volatile __bit TXB1DLC2 __attribute__((address(0x78AA)));


extern volatile __bit TXB1DLC3 __attribute__((address(0x78AB)));


extern volatile __bit TXB1EID0 __attribute__((address(0x78A0)));


extern volatile __bit TXB1EID1 __attribute__((address(0x78A1)));


extern volatile __bit TXB1EID10 __attribute__((address(0x789A)));


extern volatile __bit TXB1EID11 __attribute__((address(0x789B)));


extern volatile __bit TXB1EID12 __attribute__((address(0x789C)));


extern volatile __bit TXB1EID13 __attribute__((address(0x789D)));


extern volatile __bit TXB1EID14 __attribute__((address(0x789E)));


extern volatile __bit TXB1EID15 __attribute__((address(0x789F)));


extern volatile __bit TXB1EID16 __attribute__((address(0x7890)));


extern volatile __bit TXB1EID17 __attribute__((address(0x7891)));


extern volatile __bit TXB1EID2 __attribute__((address(0x78A2)));


extern volatile __bit TXB1EID3 __attribute__((address(0x78A3)));


extern volatile __bit TXB1EID4 __attribute__((address(0x78A4)));


extern volatile __bit TXB1EID5 __attribute__((address(0x78A5)));


extern volatile __bit TXB1EID6 __attribute__((address(0x78A6)));


extern volatile __bit TXB1EID7 __attribute__((address(0x78A7)));


extern volatile __bit TXB1EID8 __attribute__((address(0x7898)));


extern volatile __bit TXB1EID9 __attribute__((address(0x7899)));


extern volatile __bit TXB1ERR __attribute__((address(0x7884)));


extern volatile __bit TXB1EXIDE __attribute__((address(0x7893)));


extern volatile __bit TXB1IF __attribute__((address(0x7BBB)));


extern volatile __bit TXB1IP __attribute__((address(0x7BC3)));


extern volatile __bit TXB1LARB __attribute__((address(0x7885)));


extern volatile __bit TXB1PRI0 __attribute__((address(0x7880)));


extern volatile __bit TXB1PRI1 __attribute__((address(0x7881)));


extern volatile __bit TXB1REQ __attribute__((address(0x7883)));


extern volatile __bit TXB1RTR __attribute__((address(0x78AE)));


extern volatile __bit TXB1SID0 __attribute__((address(0x7895)));


extern volatile __bit TXB1SID1 __attribute__((address(0x7896)));


extern volatile __bit TXB1SID10 __attribute__((address(0x788F)));


extern volatile __bit TXB1SID2 __attribute__((address(0x7897)));


extern volatile __bit TXB1SID3 __attribute__((address(0x7888)));


extern volatile __bit TXB1SID4 __attribute__((address(0x7889)));


extern volatile __bit TXB1SID5 __attribute__((address(0x788A)));


extern volatile __bit TXB1SID6 __attribute__((address(0x788B)));


extern volatile __bit TXB1SID7 __attribute__((address(0x788C)));


extern volatile __bit TXB1SID8 __attribute__((address(0x788D)));


extern volatile __bit TXB1SID9 __attribute__((address(0x788E)));


extern volatile __bit TXB2ABT __attribute__((address(0x7806)));


extern volatile __bit TXB2D00 __attribute__((address(0x7830)));


extern volatile __bit TXB2D01 __attribute__((address(0x7831)));


extern volatile __bit TXB2D02 __attribute__((address(0x7832)));


extern volatile __bit TXB2D03 __attribute__((address(0x7833)));


extern volatile __bit TXB2D04 __attribute__((address(0x7834)));


extern volatile __bit TXB2D05 __attribute__((address(0x7835)));


extern volatile __bit TXB2D06 __attribute__((address(0x7836)));


extern volatile __bit TXB2D07 __attribute__((address(0x7837)));


extern volatile __bit TXB2D10 __attribute__((address(0x7838)));


extern volatile __bit TXB2D11 __attribute__((address(0x7839)));


extern volatile __bit TXB2D12 __attribute__((address(0x783A)));


extern volatile __bit TXB2D13 __attribute__((address(0x783B)));


extern volatile __bit TXB2D14 __attribute__((address(0x783C)));


extern volatile __bit TXB2D15 __attribute__((address(0x783D)));


extern volatile __bit TXB2D16 __attribute__((address(0x783E)));


extern volatile __bit TXB2D17 __attribute__((address(0x783F)));


extern volatile __bit TXB2D20 __attribute__((address(0x7840)));


extern volatile __bit TXB2D21 __attribute__((address(0x7841)));


extern volatile __bit TXB2D22 __attribute__((address(0x7842)));


extern volatile __bit TXB2D23 __attribute__((address(0x7843)));


extern volatile __bit TXB2D24 __attribute__((address(0x7844)));


extern volatile __bit TXB2D25 __attribute__((address(0x7845)));


extern volatile __bit TXB2D26 __attribute__((address(0x7846)));


extern volatile __bit TXB2D27 __attribute__((address(0x7847)));


extern volatile __bit TXB2D30 __attribute__((address(0x7848)));


extern volatile __bit TXB2D31 __attribute__((address(0x7849)));


extern volatile __bit TXB2D32 __attribute__((address(0x784A)));


extern volatile __bit TXB2D33 __attribute__((address(0x784B)));


extern volatile __bit TXB2D34 __attribute__((address(0x784C)));


extern volatile __bit TXB2D35 __attribute__((address(0x784D)));


extern volatile __bit TXB2D36 __attribute__((address(0x784E)));


extern volatile __bit TXB2D37 __attribute__((address(0x784F)));


extern volatile __bit TXB2D40 __attribute__((address(0x7850)));


extern volatile __bit TXB2D41 __attribute__((address(0x7851)));


extern volatile __bit TXB2D42 __attribute__((address(0x7852)));


extern volatile __bit TXB2D43 __attribute__((address(0x7853)));


extern volatile __bit TXB2D44 __attribute__((address(0x7854)));


extern volatile __bit TXB2D45 __attribute__((address(0x7855)));


extern volatile __bit TXB2D46 __attribute__((address(0x7856)));


extern volatile __bit TXB2D47 __attribute__((address(0x7857)));


extern volatile __bit TXB2D50 __attribute__((address(0x7858)));


extern volatile __bit TXB2D51 __attribute__((address(0x7859)));


extern volatile __bit TXB2D52 __attribute__((address(0x785A)));


extern volatile __bit TXB2D53 __attribute__((address(0x785B)));


extern volatile __bit TXB2D54 __attribute__((address(0x785C)));


extern volatile __bit TXB2D55 __attribute__((address(0x785D)));


extern volatile __bit TXB2D56 __attribute__((address(0x785E)));


extern volatile __bit TXB2D57 __attribute__((address(0x785F)));


extern volatile __bit TXB2D60 __attribute__((address(0x7860)));


extern volatile __bit TXB2D61 __attribute__((address(0x7861)));


extern volatile __bit TXB2D62 __attribute__((address(0x7862)));


extern volatile __bit TXB2D63 __attribute__((address(0x7863)));


extern volatile __bit TXB2D64 __attribute__((address(0x7864)));


extern volatile __bit TXB2D65 __attribute__((address(0x7865)));


extern volatile __bit TXB2D66 __attribute__((address(0x7866)));


extern volatile __bit TXB2D67 __attribute__((address(0x7867)));


extern volatile __bit TXB2D70 __attribute__((address(0x7868)));


extern volatile __bit TXB2D71 __attribute__((address(0x7869)));


extern volatile __bit TXB2D72 __attribute__((address(0x786A)));


extern volatile __bit TXB2D73 __attribute__((address(0x786B)));


extern volatile __bit TXB2D74 __attribute__((address(0x786C)));


extern volatile __bit TXB2D75 __attribute__((address(0x786D)));


extern volatile __bit TXB2D76 __attribute__((address(0x786E)));


extern volatile __bit TXB2D77 __attribute__((address(0x786F)));


extern volatile __bit TXB2DLC0 __attribute__((address(0x7828)));


extern volatile __bit TXB2DLC1 __attribute__((address(0x7829)));


extern volatile __bit TXB2DLC2 __attribute__((address(0x782A)));


extern volatile __bit TXB2DLC3 __attribute__((address(0x782B)));


extern volatile __bit TXB2EID0 __attribute__((address(0x7820)));


extern volatile __bit TXB2EID1 __attribute__((address(0x7821)));


extern volatile __bit TXB2EID10 __attribute__((address(0x781A)));


extern volatile __bit TXB2EID11 __attribute__((address(0x781B)));


extern volatile __bit TXB2EID12 __attribute__((address(0x781C)));


extern volatile __bit TXB2EID13 __attribute__((address(0x781D)));


extern volatile __bit TXB2EID14 __attribute__((address(0x781E)));


extern volatile __bit TXB2EID15 __attribute__((address(0x781F)));


extern volatile __bit TXB2EID16 __attribute__((address(0x7810)));


extern volatile __bit TXB2EID17 __attribute__((address(0x7811)));


extern volatile __bit TXB2EID2 __attribute__((address(0x7822)));


extern volatile __bit TXB2EID3 __attribute__((address(0x7823)));


extern volatile __bit TXB2EID4 __attribute__((address(0x7824)));


extern volatile __bit TXB2EID5 __attribute__((address(0x7825)));


extern volatile __bit TXB2EID6 __attribute__((address(0x7826)));


extern volatile __bit TXB2EID7 __attribute__((address(0x7827)));


extern volatile __bit TXB2EID8 __attribute__((address(0x7818)));


extern volatile __bit TXB2EID9 __attribute__((address(0x7819)));


extern volatile __bit TXB2ERR __attribute__((address(0x7804)));


extern volatile __bit TXB2EXIDE __attribute__((address(0x7813)));


extern volatile __bit TXB2IF __attribute__((address(0x7BBC)));


extern volatile __bit TXB2IP __attribute__((address(0x7BC4)));


extern volatile __bit TXB2LARB __attribute__((address(0x7805)));


extern volatile __bit TXB2PRI0 __attribute__((address(0x7800)));


extern volatile __bit TXB2PRI1 __attribute__((address(0x7801)));


extern volatile __bit TXB2REQ __attribute__((address(0x7803)));


extern volatile __bit TXB2RTR __attribute__((address(0x782E)));


extern volatile __bit TXB2SID0 __attribute__((address(0x7815)));


extern volatile __bit TXB2SID1 __attribute__((address(0x7816)));


extern volatile __bit TXB2SID10 __attribute__((address(0x780F)));


extern volatile __bit TXB2SID2 __attribute__((address(0x7817)));


extern volatile __bit TXB2SID3 __attribute__((address(0x7808)));


extern volatile __bit TXB2SID4 __attribute__((address(0x7809)));


extern volatile __bit TXB2SID5 __attribute__((address(0x780A)));


extern volatile __bit TXB2SID6 __attribute__((address(0x780B)));


extern volatile __bit TXB2SID7 __attribute__((address(0x780C)));


extern volatile __bit TXB2SID8 __attribute__((address(0x780D)));


extern volatile __bit TXB2SID9 __attribute__((address(0x780E)));


extern volatile __bit TXBNIE __attribute__((address(0x7D1C)));


extern volatile __bit TXBNIF __attribute__((address(0x7D24)));


extern volatile __bit TXBNIP __attribute__((address(0x7D2C)));


extern volatile __bit TXBO __attribute__((address(0x7B8D)));


extern volatile __bit TXBP __attribute__((address(0x7B8C)));


extern volatile __bit TXBnIE __attribute__((address(0x7BB4)));


extern volatile __bit TXBnIF __attribute__((address(0x7BBC)));


extern volatile __bit TXBnIP __attribute__((address(0x7BC4)));


extern volatile __bit TXCKP1 __attribute__((address(0x7D3C)));


extern volatile __bit TXCKP2 __attribute__((address(0x7DCC)));


extern volatile __bit TXD8 __attribute__((address(0x7D60)));


extern volatile __bit TXD82 __attribute__((address(0x7DD0)));


extern volatile __bit TXEN1 __attribute__((address(0x7D65)));


extern volatile __bit TXEN2 __attribute__((address(0x7DD5)));


extern volatile __bit TXIE __attribute__((address(0x7CEC)));


extern volatile __bit TXIF __attribute__((address(0x7CF4)));


extern volatile __bit TXIP __attribute__((address(0x7CFC)));


extern volatile __bit TXWARN __attribute__((address(0x7B8A)));


extern volatile __bit U1OD __attribute__((address(0x7C88)));


extern volatile __bit U2OD __attribute__((address(0x7C89)));


extern volatile __bit UA __attribute__((address(0x7E39)));


extern volatile __bit UART1MD __attribute__((address(0x7AC9)));


extern volatile __bit UART2MD __attribute__((address(0x7ACA)));


extern volatile __bit ULPEN __attribute__((address(0x7E8A)));


extern volatile __bit ULPLVL __attribute__((address(0x7E8D)));


extern volatile __bit ULPSINK __attribute__((address(0x7E89)));


extern volatile __bit ULPWUIN __attribute__((address(0x7C00)));


extern volatile __bit VCFG0 __attribute__((address(0x7E0C)));


extern volatile __bit VCFG01 __attribute__((address(0x7E0C)));


extern volatile __bit VCFG1 __attribute__((address(0x7E0D)));


extern volatile __bit VCFG11 __attribute__((address(0x7E0D)));


extern volatile __bit VDIRMAG __attribute__((address(0x7D47)));


extern volatile __bit VNCFG __attribute__((address(0x7E0B)));


extern volatile __bit WAIT0 __attribute__((address(0x7E5C)));


extern volatile __bit WAIT1 __attribute__((address(0x7E5D)));


extern volatile __bit WAKDIS __attribute__((address(0x722F)));


extern volatile __bit WAKFIL __attribute__((address(0x722E)));


extern volatile __bit WAKIE __attribute__((address(0x7BB6)));


extern volatile __bit WAKIF __attribute__((address(0x7BBE)));


extern volatile __bit WAKIP __attribute__((address(0x7BC6)));


extern volatile __bit WCOL __attribute__((address(0x7E37)));


extern volatile __bit WM0 __attribute__((address(0x7E58)));


extern volatile __bit WM1 __attribute__((address(0x7E59)));


extern volatile __bit WPUB0 __attribute__((address(0x7AD8)));


extern volatile __bit WPUB1 __attribute__((address(0x7AD9)));


extern volatile __bit WPUB2 __attribute__((address(0x7ADA)));


extern volatile __bit WPUB3 __attribute__((address(0x7ADB)));


extern volatile __bit WPUB4 __attribute__((address(0x7ADC)));


extern volatile __bit WPUB5 __attribute__((address(0x7ADD)));


extern volatile __bit WPUB6 __attribute__((address(0x7ADE)));


extern volatile __bit WPUB7 __attribute__((address(0x7ADF)));


extern volatile __bit WR __attribute__((address(0x7BF9)));


extern volatile __bit WREN __attribute__((address(0x7BFA)));


extern volatile __bit WRERR __attribute__((address(0x7BFB)));


extern volatile __bit WUE1 __attribute__((address(0x7D39)));


extern volatile __bit WUE2 __attribute__((address(0x7DC9)));


extern volatile __bit ZERO __attribute__((address(0x7EC2)));


extern volatile __bit nA __attribute__((address(0x7E3D)));


extern volatile __bit nADDRESS __attribute__((address(0x7E3D)));


extern volatile __bit nBOR __attribute__((address(0x7E80)));


extern volatile __bit nCM __attribute__((address(0x7E85)));


extern volatile __bit nDONE __attribute__((address(0x7E11)));


extern volatile __bit nFIFOEMPTY __attribute__((address(0x7B8F)));


extern volatile __bit nPD __attribute__((address(0x7E82)));


extern volatile __bit nPOR __attribute__((address(0x7E81)));


extern volatile __bit nRBPU __attribute__((address(0x7F8F)));


extern volatile __bit nRC8 __attribute__((address(0x7D5E)));


extern volatile __bit nRI __attribute__((address(0x7E84)));


extern volatile __bit nT1DONE __attribute__((address(0x7D53)));


extern volatile __bit nT1SYNC __attribute__((address(0x7E6A)));


extern volatile __bit nT3DONE __attribute__((address(0x7D83)));


extern volatile __bit nT3SYNC __attribute__((address(0x7D8A)));


extern volatile __bit nTO __attribute__((address(0x7E83)));


extern volatile __bit nTX8 __attribute__((address(0x7D66)));


extern volatile __bit nW __attribute__((address(0x7E3A)));


extern volatile __bit nWRITE __attribute__((address(0x7E3A)));
# 141 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include/pic18_chip_select.h" 2 3
# 6 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include/pic18.h" 2 3
# 15 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include/pic18.h" 3
__attribute__((__unsupported__("The " "flash_write" " routine is no longer supported. Please use the MPLAB X MCC."))) void flash_write(const unsigned char * a, unsigned int b, __far unsigned char * c);
__attribute__((__unsupported__("The " "EraseFlash" " routine is no longer supported. Please use the MPLAB X MCC."))) void EraseFlash(unsigned long startaddr, unsigned long endaddr);






# 1 "C:\\Program Files\\Microchip\\xc8\\v3.10\\pic\\include/errata.h" 1 3
# 24 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include/pic18.h" 2 3
# 141 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include/pic18.h" 3
__attribute__((__unsupported__("The " "Read_b_eep" " routine is no longer supported. Please use the MPLAB X MCC."))) unsigned char Read_b_eep(unsigned int badd);
__attribute__((__unsupported__("The " "Busy_eep" " routine is no longer supported. Please use the MPLAB X MCC."))) void Busy_eep(void);
__attribute__((__unsupported__("The " "Write_b_eep" " routine is no longer supported. Please use the MPLAB X MCC."))) void Write_b_eep(unsigned int badd, unsigned char bdat);
# 177 "C:/Users/ianwh/.mchp_packs/Microchip/PIC18F-K_DFP/1.17.312/xc8\\pic\\include/pic18.h" 3
unsigned char __t1rd16on(void);
unsigned char __t3rd16on(void);
# 34 "C:\\Program Files\\Microchip\\xc8\\v3.10\\pic\\include/xc.h" 2 3
# 166 "../main.c" 2

# 1 "../hwsettings.h" 1
# 50 "../hwsettings.h"
# 1 "../../CBUSlib/devincs.h" 1
# 51 "../hwsettings.h" 2
# 1 "../../CBUSlib/GenericTypeDefs.h" 1
# 20 "../../CBUSlib/GenericTypeDefs.h"
    typedef unsigned char BYTE;
    typedef unsigned int WORD;
    typedef unsigned char BOOL;
    typedef unsigned long DWORD;
# 52 "../hwsettings.h" 2
# 100 "../hwsettings.h"
extern BYTE clkMHz;
# 120 "../hwsettings.h"
void setclkMHz( void );
# 168 "../main.c" 2
# 1 "../main.h" 1
# 169 "../main.c" 2
# 1 "../../VLCB-defs/vlcbdefs.h" 1
# 170 "../main.c" 2


const char bl_version[] = { 'B','L','_','V','E','R','S','I','O','N','=', 3, 3};




# 1 "../bl_romops.h" 1
# 15 "../bl_romops.h"
extern void initRomops(void);
extern unsigned char ee_read(void);
extern void ee_write(unsigned char data);

extern void flushFlash(void);
extern void eraseFlash(void);
extern unsigned char readFlashByte(void);
extern void writeFlashByte(unsigned char value);
extern void writeConfigByte(unsigned char value);
# 178 "../main.c" 2
# 195 "../main.c"
    __asm("PSECT HiVector,class=CODE,delta=1,abs");
    __asm("ORG " "0x0008");
    __asm("goto " "0x0808");

    __asm("PSECT LoVector,class=CODE,delta=1,abs");
    __asm("ORG " "0x0018");
    __asm("goto " "0x0818");
# 392 "../main.c"
typedef union {
    struct {
        unsigned char l;
        unsigned char h;
        unsigned char u;
    } ;
    uint24_t triple;
} Integer24;




typedef union {
    struct {
        unsigned char l;
        unsigned char h;
    } ;
    uint16_t word;
} Integer16;


void canInit(void);
void canSendMessage(uint8_t cdBit);


typedef struct ControlFrame {
    Integer24 bootAddress;
    unsigned char _unused0;
    unsigned char bootControlBits;
    unsigned char bootSpecialCommand;
    Integer16 bootPCChecksum ;
} ControlFrame;






ControlFrame controlFrame;
unsigned char * controlFramePtr;

typedef struct DataFrame {
    unsigned char data[8];
}DataFrame;

unsigned char frameLength;
Integer16 ourChecksum;
unsigned char verified;
unsigned char w;
volatile unsigned char * bufferPtr;
unsigned char addrL;
unsigned char addrH;
unsigned long t1;

unsigned char t2;
# 464 "../main.c"
void main(void) {

    clkMHz = 16;
# 475 "../main.c"
    BRGCON1 = (16/4)-1;


    ANCON0 = 0;
    ANCON1 = 0;







    {TRISBbits.TRISB6=TRISBbits.TRISB7=0,TRISAbits.TRISA2=1;};


    EEADR = 0xFF;
    EEADRH = 0xFF;
# 501 "../main.c"
    if ((ee_read() == 0) && (PORTAbits.RA2)) {

        __asm("goto " "0x0800");

    }
    if (!(PORTAbits.RA2)) {



        t1 = 0;
        while (!(PORTAbits.RA2)) {
            if (t1 > 0xFFFE) {
                __asm("goto " "0x0800");
            }
            t2=0;
            while (t2 < 0x07) {
                t2++;
            }
            t1++;
        }
    }


    clkMHz = 64;

    OSCTUNEbits.PLLEN = 1;
    BRGCON1 = (clkMHz/4 -1);
# 542 "../main.c"
    controlFrame.bootSpecialCommand = 0x00;
    controlFrame.bootControlBits = 0x04 | 0x08 | 0x10;


    TRISBbits.TRISB6 = 0;
    TRISBbits.TRISB7 = 0;

    LATBbits.LATB6 = 1;
    LATBbits.LATB7 = 1;


    canInit();

    initRomops();

    ourChecksum.word = 0;
    verified = 0x00;







    while (1) {
        __asm(" clrwdt");

        RXB0CONbits.RXFUL = 0;

        while(RXB0CONbits.RXFUL == 0)
            ;

        frameLength = RXB0DLC & 0x0F;
# 586 "../main.c"
        if (RXB0EIDLbits.RXB0EID0) {



            verified = 0x00;





            TBLPTRU = controlFrame.bootAddress.u & 0x1F;
            TBLPTRH = controlFrame.bootAddress.h;
            TBLPTRL = controlFrame.bootAddress.l;
            EEADRH = TBLPTRH;
            EEADR = TBLPTRL;
# 609 "../main.c"
            if (controlFrame.bootControlBits & 0x08) {
                controlFrame.bootAddress.triple += frameLength;
            }



            if ((controlFrame.bootAddress.u & (unsigned char)0xF0) <= (unsigned char)0x00) {





                if (RXB0EIDLbits.RXB0EID1) {


                    for (w=0; w<= 8; w++) {
                        EECON1 = 0x80;
                        ((DataFrame *)(&TXB0D0))->data[w] = readFlashByte();
                        TBLPTRL++;
                        if (TBLPTRL==0) {
                            TBLPTRH++;
                            if (TBLPTRH==0) {
                                TBLPTRU++;
                            }
                        }
                    }
                    TXB0DLC = 8;
# 655 "../main.c"
                    canSendMessage(1);
                } else {


                    if (TBLPTRH >= (unsigned char)0x08)




                    {

                        if ( controlFrame.bootControlBits & 0x02) {
                            eraseFlash();
                        } else {





                            for (w=0; w<frameLength; w++) {

                                writeFlashByte(((DataFrame*)&RXB0D0)->data[w]);
                                ourChecksum.word += (((DataFrame*)&RXB0D0)->data[w]);
                                TBLPTRL++;
                                if (TBLPTRL==0) {
                                    TBLPTRH++;
                                    if (TBLPTRH==0) {
                                        TBLPTRU++;
                                    }
                                }
# 698 "../main.c"
                            }
                        }
                    }





                }
            }

            if ((controlFrame.bootAddress.u & (unsigned char)0xF0) == (unsigned char)0x30) {





                if (RXB0EIDLbits.RXB0EID1) {



                    for (w=0; w<= 8; w++) {
                        EECON1 = 0xC0;
                        ((DataFrame *)(&TXB0D0))->data[w] = readFlashByte();
                        TBLPTRL++;
                        if (TBLPTRL==0) {
                             TBLPTRH++;
                        }
                    }
                    TXB0DLC = 8;
# 746 "../main.c"
                    canSendMessage(1);
                } else {

                    for (w=0; w<frameLength; w++) {

                        __asm(" clrwdt");







                        writeConfigByte(((DataFrame*)&RXB0D0)->data[w]);
                        ourChecksum.word += (((DataFrame*)&RXB0D0)->data[w]);
                        TBLPTRL++;
                        if (TBLPTRL==0) {
                             TBLPTRH++;
                        }
# 775 "../main.c"
                    }
                }
            }
            if ((controlFrame.bootAddress.u & (unsigned char)0xF0) == (unsigned char)0xF0) {





                if (RXB0EIDLbits.RXB0EID1) {


                    for (w=0; w<= 8; w++) {
                        ((DataFrame *)(&TXB0D0))->data[w] = ee_read();
                        EEADR++;
                        if (EEADR == 0) EEADRH++;
                    }
                    TXB0DLC = 8;
# 807 "../main.c"
                    canSendMessage(1);
                } else {


                    for (w=0; w<frameLength; w++) {
                        ee_write(((DataFrame*)&RXB0D0)->data[w]);
                        ourChecksum.word += (((DataFrame*)&RXB0D0)->data[w]);
                        EEADR++;
                        if (EEADR == 0) EEADRH++;
                    }
# 828 "../main.c"
                }
            }


            if ((! RXB0EIDLbits.RXB0EID1) && (controlFrame.bootControlBits & 0x10)) {





                TXB0D0 = 0x05;

                TXB0DLC = 1;
# 853 "../main.c"
                canSendMessage(0);
            }

        } else {
# 865 "../main.c"
            flushFlash();

            controlFramePtr = (unsigned char*)&controlFrame;

            bufferPtr = &RXB0D0;




            for (w=0; w<frameLength; w++) {
                *controlFramePtr = *bufferPtr;
                controlFramePtr++;
                bufferPtr++;
            }
# 891 "../main.c"
            if (controlFrame.bootSpecialCommand == 0x01) {

                if (verified == 0x01) {


                    EEADR = 0xFF;
                    EEADRH = 0xFF;






                    ee_write(0);
                    LATBbits.LATB6 = 0;
                    LATBbits.LATB7 = 0;
                    __asm("reset");
                }
            }





            if (controlFrame.bootSpecialCommand == 0x02) {



                ourChecksum.word = 0;
                verified = 0x00;



            }







            if (controlFrame.bootSpecialCommand == 0x03) {





                verified = ((ourChecksum.word + controlFrame.bootPCChecksum.word) != 0) ?
                    0x00 : 0x01;

                TXB0D0 = verified;
                TXB0DLC = 1;
# 958 "../main.c"
                canSendMessage(0);
            }





            if (controlFrame.bootSpecialCommand == 0x04) {


                TXB0D0 = 0x02;
                TXB0DLC = 1;
# 978 "../main.c"
                canSendMessage(0);
            }

        }




    }
}



void canSendMessage(uint8_t cdBit) {


    while (TXB0CONbits.TXREQ) {
        ;
    }
 TXB0SIDH = 0b10000000;
 TXB0SIDL = 0b00001000;
 TXB0EIDH = 0b00000000;
 TXB0EIDL = 0b00000100;

 if (cdBit) {
        TXB0EIDLbits.TXB0EID0 = 1;
    } else {
        TXB0EIDLbits.TXB0EID0 = 0;
    }
# 1023 "../main.c"
 TXB0CONbits.TXREQ = 1;




}


void canInit(void) {
    IPR5 = 0;


    CANCON = 0b10000000;

    while (CANSTATbits.OPMODE2 == 0)
        ;

    ECANCON = 0;
# 1053 "../main.c"
    BRGCON2 = 0b10011110;
    BRGCON3 = 0b00000011;

    CIOCON = 0b00100000;



 RXF0SIDH = 0b00000000;
 RXF0SIDL = 0b00001000;

    RXF1SIDL = 0xF3;
 RXF0EIDH = 0b00000000;
 RXF0EIDL = 0b00000111;

 RXM0SIDH = 0b11111111;
 RXM0SIDL = 0b11101011;
 RXM0EIDH = 0b11111111;
 RXM0EIDL = 0b11111000;

    RXFCON0 = 1;

 BRGCON2 = 0b10011110;
 BRGCON3 = 0b00000011;

    BIE0 = 0;
    CANCON = 0;
}
