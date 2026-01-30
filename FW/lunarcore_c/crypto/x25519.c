/* ## Translation Notes

### Type Conversions
1. **Struct Definition**: Rust's `struct Fe([i64; 10])` with `#[derive(Clone, Copy)]` translates to a C `typedef struct` containing an array member named `limbs`. The Clone and Copy traits are implicit in C for this struct type.

2. **Integer Types**: 
   - `i64` → `int64_t` (requires `<stdint.h>`)
   - `i128` → `__int128` (GCC/Clang extension, widely supported)
   - `u8` → `uint8_t`

3. **Const Functions**: Rust's `const fn` translates to regular `static` functions in C, as C doesn't have compile-time function evaluation in the same way.

### Memory and Array Handling
1. **Array Slicing**: Rust's slice syntax `&bytes[0..4]` becomes pointer arithmetic `&bytes[0]` in C.

2. **Array Initialization**: Rust's `[0; 10]` syntax is replaced with explicit loops or memcpy operations.

3. **Return by Value**: C returns structs by value just like Rust, so the translation is straightforward.

### Function Naming Conventions
1. **Methods to Functions**: Rust impl methods become C functions with an `fe_` prefix:
   - `Fe::zero()` → `fe_zero()`
   - `self.add(rhs)` → `fe_add(a, b)`
   - `Fe::cswap(a, b, swap)` → `fe_cswap(a, b, swap)`

2. **Self Parameter**: Rust's `&self` becomes `const Fe *a` (first parameter) in C.

3. **Mutable References**: Rust's `&mut Fe` becomes `Fe *` in C.

### Specific Implementation Details
1. **Carry Propagation**: The `carry_mul` function uses `__int128` for intermediate calculations to prevent overflow, exactly as in the Rust version.

2. **Constant-Time Swap**: The `fe_cswap` function maintains constant-time behavior using bitwise operations, identical to the original.

3. **Public Functions**: The `x25519` and `x25519_base` functions are not marked `static` as they correspond to Rust's `pub` functions.

4. **Array Copying**: Used `memcpy` for efficient array copying where appropriate, which is standard practice in C.

### Arithmetic Operations
1. All arithmetic operations are preserved exactly as in the original, including:
   - The specific bit shifts and masks in `from_bytes` and `to_bytes`
   - The multiplication algorithm with precomputed constants (19, 121666)
   - The carry propagation logic
   - The inversion algorithm using Fermat's little theorem

### Dependencies
The translation requires:
- `<stdint.h>` for fixed-width integer types
- `<string.h>` for memcpy
- Compiler support for `__int128` (GCC, Clang, ICC)

### Platform Considerations
This code assumes:
- Little-endian byte order (matches Rust implementation)
- Two's complement representation for signed integers
- `__int128` support (available on 64-bit platforms with modern compilers)

If `__int128` is not available, an alternative implementation would need to use 64-bit limbs with manual carry handling, but this would significantly complicate the code.

### Comments
All significant implementation decisions are preserved through the structure of the code. The original Rust code had minimal comments, so this translation maintains that characteristic while the function names and structure remain self-documenting.
 */

#include <stdint.h>
#include <string.h>

// Structure representing a field element with 10 limbs
typedef struct {
    int64_t limbs[10];
} Fe;

// Constant P array
static const int64_t P[10] = {
    0x3ffffed, 0x1ffffff, 0x3ffffff, 0x1ffffff, 0x3ffffff,
    0x1ffffff, 0x3ffffff, 0x1ffffff, 0x3ffffff, 0x1ffffff
};

// Forward declarations of helper functions
static int64_t load_4(const uint8_t *s);
static int64_t load_3(const uint8_t *s);
static Fe carry_mul(__int128 h[10]);

// Creates a zero field element
static Fe fe_zero(void) {
    Fe result;
    for (int i = 0; i < 10; i++) {
        result.limbs[i] = 0;
    }
    return result;
}

// Creates a field element representing one
static Fe fe_one(void) {
    Fe result;
    result.limbs[0] = 1;
    for (int i = 1; i < 10; i++) {
        result.limbs[i] = 0;
    }
    return result;
}

// Deserializes a field element from 32 bytes
static Fe fe_from_bytes(const uint8_t bytes[32]) {
    int64_t h[10];

    h[0] = load_4(&bytes[0]) & 0x3ffffff;
    h[1] = (load_4(&bytes[3]) >> 2) & 0x1ffffff;
    h[2] = (load_4(&bytes[6]) >> 3) & 0x3ffffff;
    h[3] = (load_4(&bytes[9]) >> 5) & 0x1ffffff;
    h[4] = (load_4(&bytes[12]) >> 6) & 0x3ffffff;
    h[5] = (load_4(&bytes[16])) & 0x1ffffff;
    h[6] = (load_4(&bytes[19]) >> 1) & 0x3ffffff;
    h[7] = (load_4(&bytes[22]) >> 3) & 0x1ffffff;
    h[8] = (load_4(&bytes[25]) >> 4) & 0x3ffffff;
    h[9] = (load_3(&bytes[28]) >> 6) & 0x1ffffff;

    Fe result;
    memcpy(result.limbs, h, sizeof(h));
    return result;
}

// Serializes a field element to 32 bytes
static void fe_to_bytes(uint8_t out[32], const Fe *fe) {
    int64_t h[10];
    memcpy(h, fe->limbs, sizeof(h));

    int64_t q = (19 * h[9] + (1 << 24)) >> 25;
    q = (h[0] + q) >> 26;
    q = (h[1] + q) >> 25;
    q = (h[2] + q) >> 26;
    q = (h[3] + q) >> 25;
    q = (h[4] + q) >> 26;
    q = (h[5] + q) >> 25;
    q = (h[6] + q) >> 26;
    q = (h[7] + q) >> 25;
    q = (h[8] + q) >> 26;
    q = (h[9] + q) >> 25;

    h[0] += 19 * q;

    int64_t carry0 = h[0] >> 26;
    h[1] += carry0;
    h[0] -= carry0 << 26;
    int64_t carry1 = h[1] >> 25;
    h[2] += carry1;
    h[1] -= carry1 << 25;
    int64_t carry2 = h[2] >> 26;
    h[3] += carry2;
    h[2] -= carry2 << 26;
    int64_t carry3 = h[3] >> 25;
    h[4] += carry3;
    h[3] -= carry3 << 25;
    int64_t carry4 = h[4] >> 26;
    h[5] += carry4;
    h[4] -= carry4 << 26;
    int64_t carry5 = h[5] >> 25;
    h[6] += carry5;
    h[5] -= carry5 << 25;
    int64_t carry6 = h[6] >> 26;
    h[7] += carry6;
    h[6] -= carry6 << 26;
    int64_t carry7 = h[7] >> 25;
    h[8] += carry7;
    h[7] -= carry7 << 25;
    int64_t carry8 = h[8] >> 26;
    h[9] += carry8;
    h[8] -= carry8 << 26;
    int64_t carry9 = h[9] >> 25;
    h[9] -= carry9 << 25;

    uint8_t s[32];
    s[0] = (uint8_t)h[0];
    s[1] = (uint8_t)(h[0] >> 8);
    s[2] = (uint8_t)(h[0] >> 16);
    s[3] = (uint8_t)((h[0] >> 24) | (h[1] << 2));
    s[4] = (uint8_t)(h[1] >> 6);
    s[5] = (uint8_t)(h[1] >> 14);
    s[6] = (uint8_t)((h[1] >> 22) | (h[2] << 3));
    s[7] = (uint8_t)(h[2] >> 5);
    s[8] = (uint8_t)(h[2] >> 13);
    s[9] = (uint8_t)((h[2] >> 21) | (h[3] << 5));
    s[10] = (uint8_t)(h[3] >> 3);
    s[11] = (uint8_t)(h[3] >> 11);
    s[12] = (uint8_t)((h[3] >> 19) | (h[4] << 6));
    s[13] = (uint8_t)(h[4] >> 2);
    s[14] = (uint8_t)(h[4] >> 10);
    s[15] = (uint8_t)(h[4] >> 18);
    s[16] = (uint8_t)h[5];
    s[17] = (uint8_t)(h[5] >> 8);
    s[18] = (uint8_t)(h[5] >> 16);
    s[19] = (uint8_t)((h[5] >> 24) | (h[6] << 1));
    s[20] = (uint8_t)(h[6] >> 7);
    s[21] = (uint8_t)(h[6] >> 15);
    s[22] = (uint8_t)((h[6] >> 23) | (h[7] << 3));
    s[23] = (uint8_t)(h[7] >> 5);
    s[24] = (uint8_t)(h[7] >> 13);
    s[25] = (uint8_t)((h[7] >> 21) | (h[8] << 4));
    s[26] = (uint8_t)(h[8] >> 4);
    s[27] = (uint8_t)(h[8] >> 12);
    s[28] = (uint8_t)((h[8] >> 20) | (h[9] << 6));
    s[29] = (uint8_t)(h[9] >> 2);
    s[30] = (uint8_t)(h[9] >> 10);
    s[31] = (uint8_t)(h[9] >> 18);

    memcpy(out, s, 32);
}

// Field element addition
static Fe fe_add(const Fe *a, const Fe *b) {
    Fe result;
    result.limbs[0] = a->limbs[0] + b->limbs[0];
    result.limbs[1] = a->limbs[1] + b->limbs[1];
    result.limbs[2] = a->limbs[2] + b->limbs[2];
    result.limbs[3] = a->limbs[3] + b->limbs[3];
    result.limbs[4] = a->limbs[4] + b->limbs[4];
    result.limbs[5] = a->limbs[5] + b->limbs[5];
    result.limbs[6] = a->limbs[6] + b->limbs[6];
    result.limbs[7] = a->limbs[7] + b->limbs[7];
    result.limbs[8] = a->limbs[8] + b->limbs[8];
    result.limbs[9] = a->limbs[9] + b->limbs[9];
    return result;
}

// Field element subtraction
static Fe fe_sub(const Fe *a, const Fe *b) {
    Fe result;
    result.limbs[0] = a->limbs[0] - b->limbs[0];
    result.limbs[1] = a->limbs[1] - b->limbs[1];
    result.limbs[2] = a->limbs[2] - b->limbs[2];
    result.limbs[3] = a->limbs[3] - b->limbs[3];
    result.limbs[4] = a->limbs[4] - b->limbs[4];
    result.limbs[5] = a->limbs[5] - b->limbs[5];
    result.limbs[6] = a->limbs[6] - b->limbs[6];
    result.limbs[7] = a->limbs[7] - b->limbs[7];
    result.limbs[8] = a->limbs[8] - b->limbs[8];
    result.limbs[9] = a->limbs[9] - b->limbs[9];
    return result;
}

// Field element multiplication
static Fe fe_mul(const Fe *a, const Fe *b) {
    const int64_t *f = a->limbs;
    const int64_t *g = b->limbs;

    __int128 f0 = (__int128)f[0];
    __int128 f1 = (__int128)f[1];
    __int128 f2 = (__int128)f[2];
    __int128 f3 = (__int128)f[3];
    __int128 f4 = (__int128)f[4];
    __int128 f5 = (__int128)f[5];
    __int128 f6 = (__int128)f[6];
    __int128 f7 = (__int128)f[7];
    __int128 f8 = (__int128)f[8];
    __int128 f9 = (__int128)f[9];

    __int128 g0 = (__int128)g[0];
    __int128 g1 = (__int128)g[1];
    __int128 g2 = (__int128)g[2];
    __int128 g3 = (__int128)g[3];
    __int128 g4 = (__int128)g[4];
    __int128 g5 = (__int128)g[5];
    __int128 g6 = (__int128)g[6];
    __int128 g7 = (__int128)g[7];
    __int128 g8 = (__int128)g[8];
    __int128 g9 = (__int128)g[9];

    __int128 g1_19 = 19 * g1;
    __int128 g2_19 = 19 * g2;
    __int128 g3_19 = 19 * g3;
    __int128 g4_19 = 19 * g4;
    __int128 g5_19 = 19 * g5;
    __int128 g6_19 = 19 * g6;
    __int128 g7_19 = 19 * g7;
    __int128 g8_19 = 19 * g8;
    __int128 g9_19 = 19 * g9;

    __int128 f1_2 = 2 * f1;
    __int128 f3_2 = 2 * f3;
    __int128 f5_2 = 2 * f5;
    __int128 f7_2 = 2 * f7;
    __int128 f9_2 = 2 * f9;

    __int128 h[10];
    h[0] = f0 * g0 + f1_2 * g9_19 + f2 * g8_19 + f3_2 * g7_19 + f4 * g6_19 + f5_2 * g5_19 + f6 * g4_19 + f7_2 * g3_19 + f8 * g2_19 + f9_2 * g1_19;
    h[1] = f0 * g1 + f1 * g0 + f2 * g9_19 + f3 * g8_19 + f4 * g7_19 + f5 * g6_19 + f6 * g5_19 + f7 * g4_19 + f8 * g3_19 + f9 * g2_19;
    h[2] = f0 * g2 + f1_2 * g1 + f2 * g0 + f3_2 * g9_19 + f4 * g8_19 + f5_2 * g7_19 + f6 * g6_19 + f7_2 * g5_19 + f8 * g4_19 + f9_2 * g3_19;
    h[3] = f0 * g3 + f1 * g2 + f2 * g1 + f3 * g0 + f4 * g9_19 + f5 * g8_19 + f6 * g7_19 + f7 * g6_19 + f8 * g5_19 + f9 * g4_19;
    h[4] = f0 * g4 + f1_2 * g3 + f2 * g2 + f3_2 * g1 + f4 * g0 + f5_2 * g9_19 + f6 * g8_19 + f7_2 * g7_19 + f8 * g6_19 + f9_2 * g5_19;
    h[5] = f0 * g5 + f1 * g4 + f2 * g3 + f3 * g2 + f4 * g1 + f5 * g0 + f6 * g9_19 + f7 * g8_19 + f8 * g7_19 + f9 * g6_19;
    h[6] = f0 * g6 + f1_2 * g5 + f2 * g4 + f3_2 * g3 + f4 * g2 + f5_2 * g1 + f6 * g0 + f7_2 * g9_19 + f8 * g8_19 + f9_2 * g7_19;
    h[7] = f0 * g7 + f1 * g6 + f2 * g5 + f3 * g4 + f4 * g3 + f5 * g2 + f6 * g1 + f7 * g0 + f8 * g9_19 + f9 * g8_19;
    h[8] = f0 * g8 + f1_2 * g7 + f2 * g6 + f3_2 * g5 + f4 * g4 + f5_2 * g3 + f6 * g2 + f7_2 * g1 + f8 * g0 + f9_2 * g9_19;
    h[9] = f0 * g9 + f1 * g8 + f2 * g7 + f3 * g6 + f4 * g5 + f5 * g4 + f6 * g3 + f7 * g2 + f8 * g1 + f9 * g0;

    return carry_mul(h);
}

// Field element squaring
static Fe fe_square(const Fe *a) {
    return fe_mul(a, a);
}

// Multiply field element by constant 121666
static Fe fe_mul121666(const Fe *a) {
    __int128 h[10];
    for (int i = 0; i < 10; i++) {
        h[i] = ((__int128)a->limbs[i]) * 121666;
    }
    return carry_mul(h);
}

// Field element inversion
static Fe fe_invert(const Fe *a) {
    Fe t0 = fe_square(a);
    Fe t1 = fe_square(&t0);
    t1 = fe_square(&t1);
    t1 = fe_mul(a, &t1);
    t0 = fe_mul(&t0, &t1);
    Fe t2 = fe_square(&t0);
    t1 = fe_mul(&t1, &t2);
    t2 = fe_square(&t1);
    for (int i = 1; i < 5; i++) {
        t2 = fe_square(&t2);
    }
    t1 = fe_mul(&t2, &t1);
    t2 = fe_square(&t1);
    for (int i = 1; i < 10; i++) {
        t2 = fe_square(&t2);
    }
    t2 = fe_mul(&t2, &t1);
    Fe t3 = fe_square(&t2);
    for (int i = 1; i < 20; i++) {
        t3 = fe_square(&t3);
    }
    t2 = fe_mul(&t3, &t2);
    t2 = fe_square(&t2);
    for (int i = 1; i < 10; i++) {
        t2 = fe_square(&t2);
    }
    t1 = fe_mul(&t2, &t1);
    t2 = fe_square(&t1);
    for (int i = 1; i < 50; i++) {
        t2 = fe_square(&t2);
    }
    t2 = fe_mul(&t2, &t1);
    t3 = fe_square(&t2);
    for (int i = 1; i < 100; i++) {
        t3 = fe_square(&t3);
    }
    t2 = fe_mul(&t3, &t2);
    t2 = fe_square(&t2);
    for (int i = 1; i < 50; i++) {
        t2 = fe_square(&t2);
    }
    t1 = fe_mul(&t2, &t1);
    t1 = fe_square(&t1);
    t1 = fe_square(&t1);
    t1 = fe_square(&t1);
    t1 = fe_square(&t1);
    t1 = fe_square(&t1);
    return fe_mul(&t0, &t1);
}

// Constant-time conditional swap
static void fe_cswap(Fe *a, Fe *b, int64_t swap) {
    int64_t swap_mask = -swap;
    for (int i = 0; i < 10; i++) {
        int64_t x = swap_mask & (a->limbs[i] ^ b->limbs[i]);
        a->limbs[i] ^= x;
        b->limbs[i] ^= x;
    }
}

// Helper function: Load 4 bytes as little-endian int64_t
static int64_t load_4(const uint8_t *s) {
    return (int64_t)s[0]
        | ((int64_t)s[1] << 8)
        | ((int64_t)s[2] << 16)
        | ((int64_t)s[3] << 24);
}

// Helper function: Load 3 bytes as little-endian int64_t
static int64_t load_3(const uint8_t *s) {
    return (int64_t)s[0] | ((int64_t)s[1] << 8) | ((int64_t)s[2] << 16);
}

// Helper function: Perform carry propagation for multiplication result
static Fe carry_mul(__int128 h[10]) {
    int64_t out[10];

    __int128 carry = (h[0] + ((__int128)1 << 25)) >> 26;
    out[0] = (int64_t)(h[0] - (carry << 26));
    __int128 h1 = h[1] + carry;

    carry = (h1 + ((__int128)1 << 24)) >> 25;
    out[1] = (int64_t)(h1 - (carry << 25));
    __int128 h2 = h[2] + carry;

    carry = (h2 + ((__int128)1 << 25)) >> 26;
    out[2] = (int64_t)(h2 - (carry << 26));
    __int128 h3 = h[3] + carry;

    carry = (h3 + ((__int128)1 << 24)) >> 25;
    out[3] = (int64_t)(h3 - (carry << 25));
    __int128 h4 = h[4] + carry;

    carry = (h4 + ((__int128)1 << 25)) >> 26;
    out[4] = (int64_t)(h4 - (carry << 26));
    __int128 h5 = h[5] + carry;

    carry = (h5 + ((__int128)1 << 24)) >> 25;
    out[5] = (int64_t)(h5 - (carry << 25));
    __int128 h6 = h[6] + carry;

    carry = (h6 + ((__int128)1 << 25)) >> 26;
    out[6] = (int64_t)(h6 - (carry << 26));
    __int128 h7 = h[7] + carry;

    carry = (h7 + ((__int128)1 << 24)) >> 25;
    out[7] = (int64_t)(h7 - (carry << 25));
    __int128 h8 = h[8] + carry;

    carry = (h8 + ((__int128)1 << 25)) >> 26;
    out[8] = (int64_t)(h8 - (carry << 26));
    __int128 h9 = h[9] + carry;

    carry = (h9 + ((__int128)1 << 24)) >> 25;
    out[9] = (int64_t)(h9 - (carry << 25));
    out[0] += (int64_t)(carry * 19);

    carry = ((__int128)out[0] + ((__int128)1 << 25)) >> 26;
    out[0] -= (int64_t)(carry << 26);
    out[1] += (int64_t)carry;

    Fe result;
    memcpy(result.limbs, out, sizeof(out));
    return result;
}

// X25519 scalar multiplication
void x25519(uint8_t out[32], const uint8_t scalar[32], const uint8_t point[32]) {
    uint8_t k[32];
    memcpy(k, scalar, 32);
    k[0] &= 248;
    k[31] &= 127;
    k[31] |= 64;

    Fe u = fe_from_bytes(point);

    Fe x_1 = u;
    Fe x_2 = fe_one();
    Fe z_2 = fe_zero();
    Fe x_3 = u;
    Fe z_3 = fe_one();

    int64_t swap = 0;

    for (int pos = 254; pos >= 0; pos--) {
        int64_t bit = (int64_t)((k[pos / 8] >> (pos & 7)) & 1);
        swap ^= bit;
        fe_cswap(&x_2, &x_3, swap);
        fe_cswap(&z_2, &z_3, swap);
        swap = bit;

        Fe a = fe_add(&x_2, &z_2);
        Fe aa = fe_square(&a);
        Fe b = fe_sub(&x_2, &z_2);
        Fe bb = fe_square(&b);
        Fe e = fe_sub(&aa, &bb);
        Fe c = fe_add(&x_3, &z_3);
        Fe d = fe_sub(&x_3, &z_3);
        Fe da = fe_mul(&d, &a);
        Fe cb = fe_mul(&c, &b);
        Fe sum = fe_add(&da, &cb);
        Fe diff = fe_sub(&da, &cb);
        x_3 = fe_square(&sum);
        Fe diff_sq = fe_square(&diff);
        z_3 = fe_mul(&x_1, &diff_sq);
        x_2 = fe_mul(&aa, &bb);
        Fe e_mul = fe_mul121666(&e);
        Fe aa_plus_e_mul = fe_add(&aa, &e_mul);
        z_2 = fe_mul(&e, &aa_plus_e_mul);
    }

    fe_cswap(&x_2, &x_3, swap);
    fe_cswap(&z_2, &z_3, swap);

    Fe z_2_inv = fe_invert(&z_2);
    Fe result = fe_mul(&x_2, &z_2_inv);
    fe_to_bytes(out, &result);
}

// X25519 scalar multiplication with the base point
void x25519_base(uint8_t out[32], const uint8_t scalar[32]) {
    const uint8_t basepoint[32] = {
        9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    };
    x25519(out, scalar, basepoint);
}
