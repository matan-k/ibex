/* Copyright 2014, Kenneth MacKay. Licensed under the BSD 2-clause license. */

#include "uECC.h"

// #include <stdio.h>
// #include <string.h>
#include "simple_system_common.h"
#include "strings.h"
#include "random.h"


static int RNG(uint8_t *dest, unsigned size) {
    static randctx ctx;
    static int first_round = 1;

    // Initilize random seed.
    if(first_round == 1) {
      ctx.randa=ctx.randb=ctx.randc=(ub4)0;
      for (ub4 i=0; i<256; ++i) ctx.randrsl[i]=(ub4)0;
      randinit(&ctx, TRUE);
      first_round = 0;
    }

    // Fill in the noise.
    while (size) {
      isaac(&ctx);
      *dest = ctx.randrsl;
      ++dest;
      --size;
    }
    // NOTE: it would be a good idea to hash the resulting random data using SHA-256 or similar.
    return 1;
}

void vli_print(uint8_t *vli, unsigned int size) {
    for(unsigned i=0; i<size; ++i) {
        puthex((unsigned)vli[i]);
    }
}



int main() {
    int i, c;
    uint8_t private1[32] = {0};
    uint8_t private2[32] = {0};
    uint8_t public1[64] = {0};
    uint8_t public2[64] = {0};
    uint8_t secret1[32] = {0};
    uint8_t secret2[32] = {0};

    uECC_set_rng(&RNG);
    
    const struct uECC_Curve_t * curves[5];
    int num_curves = 0;
#if uECC_SUPPORTS_secp160r1
    curves[num_curves++] = uECC_secp160r1();
#endif
#if uECC_SUPPORTS_secp192r1
    curves[num_curves++] = uECC_secp192r1();
#endif
#if uECC_SUPPORTS_secp224r1
    curves[num_curves++] = uECC_secp224r1();
#endif
#if uECC_SUPPORTS_secp256r1
    curves[num_curves++] = uECC_secp256r1();
#endif
#if uECC_SUPPORTS_secp256k1
    curves[num_curves++] = uECC_secp256k1();
#endif
    
    puts("Testing 1 random private key pairs\n");

    for (c = 0; c < num_curves; ++c) {
        for (i = 0; i < 1; ++i) {
            puts(".");
            // fflush(stdout);

            if (!uECC_make_key(public1, private1, curves[c]) ||
                !uECC_make_key(public2, private2, curves[c])) {
                puts("uECC_make_key() failed\n");
                return 1;
            }

            if (!uECC_shared_secret(public2, private1, secret1, curves[c])) {
                puts("shared_secret() failed (1)\n");
                return 1;
            }

            if (!uECC_shared_secret(public1, private2, secret2, curves[c])) {
                puts("shared_secret() failed (2)\n");
                return 1;
            }
        
            if (memcmp(secret1, secret2, sizeof(secret1)) != 0) {
                puts("Shared secrets are not identical!\n");
                puts("Private key 1 = ");
                vli_print(private1, 32);
                puts("\n");
                puts("Private key 2 = ");
                vli_print(private2, 32);
                puts("\n");
                puts("Public key 1 = ");
                vli_print(public1, 64);
                puts("\n");
                puts("Public key 2 = ");
                vli_print(public2, 64);
                puts("\n");
                puts("Shared secret 1 = ");
                vli_print(secret1, 32);
                puts("\n");
                puts("Shared secret 2 = ");
                vli_print(secret2, 32);
                puts("\n");
            }
        }
        puts("\n");
    }
    
    return 0;
}
