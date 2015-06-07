#include <stdint.h>

#define FONT_WIDTH								8
#define FONT_HEIGHT								8
#define NUMBER_OFFSET							16
#define OFFSET_FROM_ASCII					((uint8_t) 0x20)
#define NUMBER_OF_CHARACTERS			98

static const uint8_t FONT_BITMAP[98][8] = { 
{   0,   0,   0,   0,   0,   0,   0,   0}, // ' '
 {   0,   0,  96, 250, 250,  96,   0,   0}, // '!'
 {   0, 192, 192,   0, 192, 192,   0,   0}, // '"'
 {  40, 254, 254,  40, 254, 254,  40,   0}, // '#'
 {  36, 116, 214, 214,  92,  72,   0,   0}, // '$'
 {  98, 102,  12,  24,  48, 102,  70,   0}, // '%'
 {  12,  94, 242, 186, 236,  94,  18,   0}, // '&'
 {  32, 224, 192,   0,   0,   0,   0,   0}, // '''
 {   0,  56, 124, 198, 130,   0,   0,   0}, // '('
 {   0, 130, 198, 124,  56,   0,   0,   0}, // ')'
 {  16,  84, 124,  56,  56, 124,  84,  16}, // '*'
 {  16,  16, 124, 124,  16,  16,   0,   0}, // '+'
 {   0,   1,   7,   6,   0,   0,   0,   0}, // ','
 {  16,  16,  16,  16,  16,  16,   0,   0}, // '-'
 {   0,   0,   6,   6,   0,   0,   0,   0}, // '.'
 {   6,  12,  24,  48,  96, 192, 128,   0}, // '/'
 { 124, 254, 142, 154, 178, 254, 124,   0}, // '0'
 {   2,  66, 254, 254,   2,   2,   0,   0}, // '1'
 {  70, 206, 154, 146, 246, 102,   0,   0}, // '2'
 {  68, 198, 146, 146, 254, 108,   0,   0}, // '3'
 {  24,  56, 104, 202, 254, 254,  10,   0}, // '4'
 { 228, 230, 162, 162, 190, 156,   0,   0}, // '5'
 {  60, 126, 210, 146, 158,  12,   0,   0}, // '6'
 { 192, 192, 142, 158, 240, 224,   0,   0}, // '7'
 { 108, 254, 146, 146, 254, 108,   0,   0}, // '8'
 {  96, 242, 146, 150, 252, 120,   0,   0}, // '9'
 {   0,   0, 102, 102,   0,   0,   0,   0}, // ':'
 {   0,   1, 103, 102,   0,   0,   0,   0}, // ';'
 {  16,  56, 108, 198, 130,   0,   0,   0}, // '<'
 {  36,  36,  36,  36,  36,  36,   0,   0}, // '='
 {   0, 130, 198, 108,  56,  16,   0,   0}, // '>'
 {  64, 192, 138, 154, 240,  96,   0,   0}, // '?'
 { 124, 254, 130, 186, 186, 248, 120,   0}, // '@'
 {  62, 126, 200, 200, 126,  62,   0,   0}, // 'A'
 { 130, 254, 254, 146, 146, 254, 108,   0}, // 'B'
 {  56, 124, 198, 130, 130, 198,  68,   0}, // 'C'
 { 130, 254, 254, 130, 198, 124,  56,   0}, // 'D'
 { 130, 254, 254, 146, 186, 130, 198,   0}, // 'E'
 { 130, 254, 254, 146, 184, 128, 192,   0}, // 'F'
 {  56, 124, 198, 130, 138, 206,  78,   0}, // 'G'
 { 254, 254,  16,  16, 254, 254,   0,   0}, // 'H'
 {   0, 130, 254, 254, 130,   0,   0,   0}, // 'I'
 {  12,  14,   2, 130, 254, 252, 128,   0}, // 'J'
 { 130, 254, 254,  16,  56, 238, 198,   0}, // 'K'
 { 130, 254, 254, 130,   2,   6,  14,   0}, // 'L'
 { 254, 254, 112,  56, 112, 254, 254,   0}, // 'M'
 { 254, 254,  96,  48,  24, 254, 254,   0}, // 'N'
 {  56, 124, 198, 130, 198, 124,  56,   0}, // 'O'
 { 130, 254, 254, 146, 144, 240,  96,   0}, // 'P'
 { 120, 252, 132, 142, 254, 122,   0,   0}, // 'Q'
 { 130, 254, 254, 144, 152, 254, 102,   0}, // 'R'
 { 100, 246, 178, 154, 206,  76,   0,   0}, // 'S'
 { 192, 130, 254, 254, 130, 192,   0,   0}, // 'T'
 { 254, 254,   2,   2, 254, 254,   0,   0}, // 'U'
 { 248, 252,   6,   6, 252, 248,   0,   0}, // 'V'
 { 254, 254,  12,  24,  12, 254, 254,   0}, // 'W'
 { 194, 230,  60,  24,  60, 230, 194,   0}, // 'X'
 { 224, 242,  30,  30, 242, 224,   0,   0}, // 'Y'
 { 226, 198, 142, 154, 178, 230, 206,   0}, // 'Z'
 {   0, 254, 254, 130, 130,   0,   0,   0}, // '['
 { 128, 192,  96,  48,  24,  12,   6,   0}, // '\'
 {   0, 130, 130, 254, 254,   0,   0,   0}, // ']'
 {  16,  48,  96, 192,  96,  48,  16,   0}, // '^'
 {   1,   1,   1,   1,   1,   1,   1,   1}, // '_'
 {   0,   0, 192, 224,  32,   0,   0,   0}, // '`'
 {   4,  46,  42,  42,  60,  30,   2,   0}, // 'a'
 { 130, 254, 252,  18,  18,  30,  12,   0}, // 'b'
 {  28,  62,  34,  34,  54,  20,   0,   0}, // 'c'
 {  12,  30,  18, 146, 252, 254,   2,   0}, // 'd'
 {  28,  62,  42,  42,  58,  24,   0,   0}, // 'e'
 {  18, 126, 254, 146, 192,  64,   0,   0}, // 'f'
 {  25,  61,  37,  37,  31,  62,  32,   0}, // 'g'
 { 130, 254, 254,  16,  32,  62,  30,   0}, // 'h'
 {   0,  34, 190, 190,   2,   0,   0,   0}, // 'i'
 {   6,   7,   1,   1, 191, 190,   0,   0}, // 'j'
 { 130, 254, 254,   8,  28,  54,  34,   0}, // 'k'
 {   0, 130, 254, 254,   2,   0,   0,   0}, // 'l'
 {  62,  62,  24,  28,  56,  62,  30,   0}, // 'm'
 {  62,  62,  32,  32,  62,  30,   0,   0}, // 'n'
 {  28,  62,  34,  34,  62,  28,   0,   0}, // 'o'
 {  33,  63,  31,  37,  36,  60,  24,   0}, // 'p'
 {  24,  60,  36,  37,  31,  63,  33,   0}, // 'q'
 {  34,  62,  30,  50,  32,  56,  24,   0}, // 'r'
 {  18,  58,  42,  42,  46,  36,   0,   0}, // 's'
 {   0,  32, 124, 254,  34,  36,   0,   0}, // 't'
 {  60,  62,   2,   2,  60,  62,   2,   0}, // 'u'
 {  56,  60,   6,   6,  60,  56,   0,   0}, // 'v'
 {  60,  62,  14,  28,  14,  62,  60,   0}, // 'w'
 {  34,  54,  28,   8,  28,  54,  34,   0}, // 'x'
 {  57,  61,   5,   5,  63,  62,   0,   0}, // 'y'
 {  50,  38,  46,  58,  50,  38,   0,   0}, // 'z'
 {  16,  16, 124, 238, 130, 130,   0,   0}, // '{'
 {   0,   0,   0, 238, 238,   0,   0,   0}, // '|'
 { 130, 130, 238, 124,  16,  16,   0,   0}, // '}'
 {  64, 192, 128, 192,  64, 192, 128,   0}, // '~'
 {   0,   0,   0,   0,   0,   0,   0,   0}, // ''
 {   0,   0,   0,   0,   0,   0,   0,   0}, // '�'
 {   0,   0,   0,   0,   0,   0,   0,   0}, // '�'
  };