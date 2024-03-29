/* 
 * File:       hex_lut.h
 * Author:     Peter Thornton
 * Purpose:    Look Up Table (LUT) for two-character hex values
 * Created on: 10/11/2020 
 */

// This LUT is needed because the KPC-9612 Plus TNC at ground station can not
// run in transparent mode with UNPROTO AX.25, while the He-100 flight radio
// can not use anything but UNPROTO AX.25. So to send binary image data we need 
// to use ascii representation. Hex values take 2 transmitted characters per 
// binary byte, but that is better than sending decimal values, which could
// require three transmitted characters per byte.
// The LUT is used on the fly in the image dump routine to build each packet.
const char hex_lut[512] = {
    '0','0',  //  0
    '0','1',  //  1
    '0','2',  //  2
    '0','3',  //  3
    '0','4',  //  4
    '0','5',  //  5
    '0','6',  //  6
    '0','7',  //  7
    '0','8',  //  8
    '0','9',  //  9
    '0','A',  // 10
    '0','B',  // 11
    '0','C',  // 12
    '0','D',  // 13
    '0','E',  // 14
    '0','F',  // 15
    '1','0',  // 16
    '1','1',  // 17
    '1','2',  // 18
    '1','3',  // 19
    '1','4',  // 20
    '1','5',  // 21
    '1','6',  // 22
    '1','7',  // 23
    '1','8',  // 24
    '1','9',  // 25
    '1','A',  // 26
    '1','B',  // 27
    '1','C',  // 28
    '1','D',  // 29
    '1','E',  // 30
    '1','F',  // 31
    '2','0',  // 32
    '2','1',  // 33
    '2','2',  // 34
    '2','3',  // 35
    '2','4',  // 36
    '2','5',  // 37
    '2','6',  // 38
    '2','7',  // 39
    '2','8',  // 40
    '2','9',  // 41
    '2','A',  // 42
    '2','B',  // 43
    '2','C',  // 44
    '2','D',  // 45
    '2','E',  // 46
    '2','F',  // 47
    '3','0',  // 48
    '3','1',  // 49
    '3','2',  // 50
    '3','3',  // 51
    '3','4',  // 52
    '3','5',  // 53
    '3','6',  // 54
    '3','7',  // 55
    '3','8',  // 56
    '3','9',  // 57
    '3','A',  // 58
    '3','B',  // 59
    '3','C',  // 60
    '3','D',  // 61
    '3','E',  // 62
    '3','F',  // 63
    '4','0',  // 64
    '4','1',  // 65
    '4','2',  // 66
    '4','3',  // 67
    '4','4',  // 68
    '4','5',  // 69
    '4','6',  // 70
    '4','7',  // 71
    '4','8',  // 72
    '4','9',  // 73
    '4','A',  // 74
    '4','B',  // 75
    '4','C',  // 76
    '4','D',  // 77
    '4','E',  // 78
    '4','F',  // 79
    '5','0',  // 80
    '5','1',  // 81
    '5','2',  // 82
    '5','3',  // 83
    '5','4',  // 84
    '5','5',  // 85
    '5','6',  // 86
    '5','7',  // 87
    '5','8',  // 88
    '5','9',  // 89
    '5','A',  // 90
    '5','B',  // 91
    '5','C',  // 92
    '5','D',  // 93
    '5','E',  // 94
    '5','F',  // 95
    '6','0',  // 96
    '6','1',  // 97
    '6','2',  // 98
    '6','3',  // 99
    '6','4',  //100
    '6','5',  //101
    '6','6',  //102
    '6','7',  //103
    '6','8',  //104
    '6','9',  //105
    '6','A',  //106
    '6','B',  //107
    '6','C',  //108
    '6','D',  //109
    '6','E',  //110
    '6','F',  //111
    '7','0',  //112
    '7','1',  //113
    '7','2',  //114
    '7','3',  //115
    '7','4',  //116
    '7','5',  //117
    '7','6',  //118
    '7','7',  //119
    '7','8',  //120
    '7','9',  //121
    '7','A',  //122
    '7','B',  //123
    '7','C',  //124
    '7','D',  //125
    '7','E',  //126
    '7','F',  //127
    '8','0',  //128
    '8','1',  //129
    '8','2',  //130
    '8','3',  //131
    '8','4',  //132
    '8','5',  //133
    '8','6',  //134
    '8','7',  //135
    '8','8',  //136
    '8','9',  //137
    '8','A',  //138
    '8','B',  //139
    '8','C',  //140
    '8','D',  //141
    '8','E',  //142
    '8','F',  //143
    '9','0',  //144
    '9','1',  //145
    '9','2',  //146
    '9','3',  //147
    '9','4',  //148
    '9','5',  //149
    '9','6',  //150
    '9','7',  //151
    '9','8',  //152
    '9','9',  //153
    '9','A',  //154
    '9','B',  //155
    '9','C',  //156
    '9','D',  //157
    '9','E',  //158
    '9','F',  //159
    'A','0',  //160
    'A','1',  //161
    'A','2',  //162
    'A','3',  //163
    'A','4',  //164
    'A','5',  //165
    'A','6',  //166
    'A','7',  //167
    'A','8',  //168
    'A','9',  //169
    'A','A',  //170
    'A','B',  //171
    'A','C',  //172
    'A','D',  //173
    'A','E',  //174
    'A','F',  //175
    'B','0',  //176
    'B','1',  //177
    'B','2',  //178
    'B','3',  //179
    'B','4',  //180
    'B','5',  //181
    'B','6',  //182
    'B','7',  //183
    'B','8',  //184
    'B','9',  //185
    'B','A',  //186
    'B','B',  //187
    'B','C',  //188
    'B','D',  //189
    'B','E',  //190
    'B','F',  //191
    'C','0',  //192
    'C','1',  //193
    'C','2',  //194
    'C','3',  //195
    'C','4',  //196
    'C','5',  //197
    'C','6',  //198
    'C','7',  //199
    'C','8',  //200
    'C','9',  //201
    'C','A',  //202
    'C','B',  //203
    'C','C',  //204
    'C','D',  //205
    'C','E',  //206
    'C','F',  //207
    'D','0',  //208
    'D','1',  //209
    'D','2',  //210
    'D','3',  //211
    'D','4',  //212
    'D','5',  //213
    'D','6',  //214
    'D','7',  //215
    'D','8',  //216
    'D','9',  //217
    'D','A',  //218
    'D','B',  //219
    'D','C',  //220
    'D','D',  //221
    'D','E',  //222
    'D','F',  //223
    'E','0',  //224
    'E','1',  //225
    'E','2',  //226
    'E','3',  //227
    'E','4',  //228
    'E','5',  //229
    'E','6',  //230
    'E','7',  //231
    'E','8',  //232
    'E','9',  //233
    'E','A',  //234
    'E','B',  //235
    'E','C',  //236
    'E','D',  //237
    'E','E',  //238
    'E','F',  //239
    'F','0',  //240
    'F','1',  //241
    'F','2',  //242
    'F','3',  //243
    'F','4',  //244
    'F','5',  //245
    'F','6',  //246
    'F','7',  //247
    'F','8',  //248
    'F','9',  //249
    'F','A',  //250
    'F','B',  //251
    'F','C',  //252
    'F','D',  //253
    'F','E',  //254
    'F','F'   //255
};
// function prototypes 





