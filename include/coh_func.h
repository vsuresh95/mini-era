///////////////////////////////////////////////////////////////
// Helper unions
///////////////////////////////////////////////////////////////
#ifndef __COH_FUNC__
#define __COH_FUNC__

typedef union
{
  struct
  {
    unsigned int r_en   : 1;
    unsigned int r_op   : 1;
    unsigned int r_type : 2;
    unsigned int r_cid  : 4;
    unsigned int w_en   : 1;
    unsigned int w_op   : 1;
    unsigned int w_type : 2;
    unsigned int w_cid  : 4;
	uint16_t reserved: 16;
  };
  uint32_t spandex_reg;
} spandex_config_t;

typedef union
{
  struct
  {
    int32_t value_32_1;
    int32_t value_32_2;
  };
  int64_t value_64;
} spandex_token_t;


///////////////////////////////////////////////////////////////
// Choosing the read, write code and coherence register config
///////////////////////////////////////////////////////////////
#define QUAUX(X) #X
#define QU(X) QUAUX(X)

#if (IS_ESP == 1)
// ESP COHERENCE PROTOCOLS
#define READ_CODE 0x0002B30B
#define WRITE_CODE 0x0062B02B
#else
//SPANDEX COHERENCE PROTOCOLS
#if (COH_MODE == 3)
// Owner Prediction
#define READ_CODE 0x4002B30B
#define WRITE_CODE 0x2262B82B
#elif (COH_MODE == 2)
// Write-through forwarding
#define READ_CODE 0x4002B30B
#define WRITE_CODE 0x2062B02B
#elif (COH_MODE == 1)
// Baseline Spandex
#define READ_CODE 0x2002B30B
#define WRITE_CODE 0x0062B02B
#else
// Fully Coherent MESI
#define READ_CODE 0x0002B30B
#define WRITE_CODE 0x0062B02B
#endif
#endif

// ///////////////////////////////////////////////////////////////
// // Choosing the read, write code and coherence register config
// ///////////////////////////////////////////////////////////////
// #define QUAUX(X) #X
// #define QU(X) QUAUX(X)

// #if (IS_ESP == 1)
// // ESP COHERENCE PROTOCOLS
// #define READ_CODE 0x0002B30B
// #define WRITE_CODE 0x0062B02B
// spandex_config_t spandex_config;

// #if (COH_MODE == 3)
// unsigned coherence = ACC_COH_NONE;
// const char print_coh[] = "Non-Coherent DMA";
// #elif (COH_MODE == 2)
// unsigned coherence = ACC_COH_LLC;
// const char print_coh[] = "LLC-Coherent DMA";
// #elif (COH_MODE == 1)
// unsigned coherence = ACC_COH_RECALL;
// const char print_coh[] = "Coherent DMA";
// #else
// unsigned coherence = ACC_COH_FULL;
// const char print_coh[] = "Baseline MESI";
// #endif

// #else
// //SPANDEX COHERENCE PROTOCOLS
// unsigned coherence = ACC_COH_FULL;
// #if (COH_MODE == 3)
// // Owner Prediction
// #define READ_CODE 0x4002B30B
// #define WRITE_CODE 0x2262B82B
// spandex_config_t spandex_config = {.spandex_reg = 0, .r_en = 1, .r_type = 2, .w_en = 1, .w_op = 1, .w_type = 1};
// const char print_coh[] = "Owner Prediction";
// #elif (COH_MODE == 2)
// // Write-through forwarding
// #define READ_CODE 0x4002B30B
// #define WRITE_CODE 0x2062B02B
// spandex_config_t spandex_config = {.spandex_reg = 0, .r_en = 1, .r_type = 2, .w_en = 1, .w_type = 1};
// const char print_coh[] = "Write-through forwarding";
// #elif (COH_MODE == 1)
// // Baseline Spandex
// #define READ_CODE 0x2002B30B
// #define WRITE_CODE 0x0062B02B
// spandex_config_t spandex_config = {.spandex_reg = 0, .r_en = 1, .r_type = 1};
// const char print_coh[] = "Baseline Spandex (ReqV)";
// #else
// // Fully Coherent MESI
// #define READ_CODE 0x0002B30B
// #define WRITE_CODE 0x0062B02B
// spandex_config_t spandex_config= {.spandex_reg = 0};
// const char print_coh[] = "Baseline Spandex";
// #endif
// #endif



#endif
