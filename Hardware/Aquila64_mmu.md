****# Aquila 64 MMU

###### tags:`aquila`, `soc`, `mmu`

---

# Introduction

此文章記錄整合 MMU 功能的 Aquila 處理器所做的硬體修改，分為處理器核心和 MMU 兩部分。

除了此文，也建議閱讀以下文章：

1. 李沿槱：基於 RISC-V SoC 的 Virtual Memory 設計與實作（109/8）
2. 何柏緯：RISC-V 處理器的記憶體管理單元與執行權限實作與驗證（110/7）
3. 蔡鎔駿：支援 Linux 作業系統之 RISC-V 處理器電路修改與驗證（112/8）

有紀錄關於 SV32 MMU 的資訊。

---

# MMU

此段落紀錄實作 SV39 虛擬記憶體功能的 MMU 實作。

部分信號的命名和原本不同，大部分基於讓變數的用途更明確，僅有少部分為了因應虛擬記憶體功能的差異。

## TLB

SV32 虛擬記憶體中， page 的大小只有 4KiB 和 4MiB 2 種。 SV39 則是 4KiB 、 2 MiB 、和 1GiB 3 種，因此修改了原先用以標註是否為 super page 的暫存器，改為紀錄 page 的大小。

此外， page 大小種類的增加，讓判斷命中或失誤的情況變得更複雜，因此採用額外的組合電路判斷 TLB 的每個 entry 是否匹配需要轉址或是清空的位址。此外，在決定 page 的選擇與 flush 行為時也改以解碼電路撰寫，使其更容易對照 SPEC 的內容。

## PTW

PTW 電路會取得當前的 PTE 對應的 page size ，因此電路僅針對增加的 page 種類進行修改，其餘功能大致相同。

另一個修改來自已知問題 2 ，新增了 `work_for_dtlb` 信號，運作方式和 `work_for_itlb` 類似。

## MMU Arbiter

MMU 仲裁電路的行為較簡單，透過 1 個狀態機控制現在使用 data port 的對象。這部分和 SV32 沒有根本的差別。

## MMU

MMU 電路本身也沒有根本的更動，嚴格來說它只負責整合 TLB 、 PTW 、 Arbiter 。

為了因應已知問題 3 ， IMMU 設計了一個 buffer 。在 IMMU 已經完成轉址和擷取指令， fetch stage 卻處於 stall 狀態時，會將資料先送入 buffer 。在輸出資料到 fetch stage 時也會優先選擇 buffer 。另一個可行的做法是修改 MMU 的行為，使其在 load use hazard 發生時也進入 stall ，停止轉址的行為。

為了因應已知問題 4 ， DMMU 在檢查 PMP exception 時會在 AMO 指令和 store 指令時檢查 page 的 W 和 D 位元。

# Processor Core

此段落紀錄處理器核心為整合 MMU 的修改。大部分的功能與原版的 Aquila 所具有的 CSR 相同，可以參考能夠 boot linux 的 32 位元 Aquila 。

## Program Counter

整合 MMU 後， CSR 的寫入可能導致 flush ，因此 program counter 需要檢查 CSR flush 並在發生時回到 memory stage 的 PC 。此外， `sfence.vma` 指令在 memory stage 由 DMMU 執行，因此需要回到 execute stage 的 PC （電路中命名 `dec_pc_i` 實際上是 decode 給 execute 的 PC ）。

## Decode

Decode stage 需要正確處理 CSR 指令和例外，這是在 dram 版本中簡略處理的部分，此外也支援了 `ebreak` 指令。 Decode 會觸發的例外有：

1. `ecall`
2. `ebreak`
3. 非法指令
    - 未支援的指令
    - 屬於 RV32 的指令（主要用以存取 CSR）
    - 非法的移位指令
    - 非法的系統指令（包含 `mret` 、 `sret` 、以及部分非法存取 CSR 的行為）

## CSR

CSR 的修改主要配合 MMU 所需的功能，此外為了支援作業系統的運作，也新增了部分功能：

1. 修改 `mstatus` 的行為
2. 實作 `misa`
3. 修改 `mie` 和 `mip` 的行為
4. 修改 `sip` 和 `sie` 的行為
5. 修改 `scause` 的行為。
6. 實作 `sedeleg` 和 `sideleg`
7. 修改發生例外或中斷時的檢查行為

輸出的修改如下：

- 輸出 `tvm` 、 `tsr` 位元
- 輸出 MMU 的啟用信號
- 輸出分頁表所需的 PPN 和 ASID ，以及 `mxr` 和 `sum` 位元
- 輸出 CSR 的 flush 信號（在修改機器的權限後，原本擷取的指令可能是錯誤的）

## Pipeline Controller

Pipeline controll 新增了 MMU 的例外信號，此外也會偵測 `sfence` 指令的執行與各個 stage 是否發生 system jump 。

## core_top.v

處理器核心的修改有兩點：

1. 將對外存取記憶體的工作改由 MMU 負責。
2. 因應完整實作的例外信號調整狀態機的行為。

---

# 已知問題

以下是整合 MMU 過程中發現的問題。

1. flush & exception

這個問題來自處理器核心在任一 stage 發生例外後會停止運作（包含從外部擷取指令和執行指令），並讓發生例外的指令進入 writeback stage 處理。然而在各個 pipeline register 中， flush 行為的優先權比例外更高。導致 2 種情況同時發生時， pipeline 會送出錯誤的資訊到下一個 stage 。

此設計導致的錯誤，會在 MMU stage 發生 fetch page fault ，而 execute stage 發生 branch flush 時發生。由於在 fetch stage 中 flush 的優先權更高，因此發生 fetch page fault 的指令不會拉起 exception 信號。

2. `work_for_itlb` 於 `ptw.v`

這個問題來自 SV32 MMU 採用單一暫存器 `work_for_itlb` 表示「為 ITLB 服務」和「為 DTLB 服務」和「沒有進行服務」，並將後 2 種情況以值 1'b0 表示。如此便使 PTW 電路無法區分。

此設計導致的錯誤，會在 MMU stage 發生 fetch page fault ，而 memory stage 同時進行 data fetch 時發生。由於原本的電路會在有例外發生時拉起暫存器的值， PTW 電路會在處理 fetch page fault 後將 data fetch 誤認為 DTLB 發生錯誤，因而拉起錯誤的 exception 信號。

3. fetch page fault

fetch page fault 有兩種情況，第 1 個是 PTW 在查表時找到無效的 PTE ，第 2 個是在存取有效的 PTE 時發生 PMP exception 。

在啟用虛擬記憶體時， MMU 會改以 pipeline 的形式輸出資料。輸入端和輸出端需要同步，意即輸出端必須等到輸入端完成處理一併進行輸出和接收新資料。當某個位址於輸入端觸發 fetch page fault ，也會拉起 valid 信號，使輸出端的指令結果進入到 fetch stage 。

因此問題來自 fetch page fault 發生時， valid 的信號來源並非 TLB hit 或是 ignore ，而是只參考 1 個 cycle 的 xcpt_valid ，這個信號沒有參考 fetch stage 的 stalling ，因此會在 fetch stage 無法接收資料的情況下給予指令，指令會丟失。

此設計導致的錯誤，會在 MMU stage 觸發 fetch page fault ，而 decode stage 取得 load use hazard 時發生。正常來說，當 load use hazard 發生時， PC 進入 stall ，因此 MMU 會維持 1 個 cycle 的 valid ，讓 fetch stage 可以暫停 1 個 cycle 後拿到一樣的位址。

4. store/amo page fault

這個問題來自 DMMU 在檢查 PMP exception 時，沒有檢查 AMO 指令。 AMO 指令在寫入 page 時，應該要檢查 D 位元是否有效。

此設計會導致 AMO 指令修改 page 時不檢查 D 位元，在 riscv-test 中會在 `evict()` 檢查出來。

# 資源使用

使用 Vivado 2024.1 在 xc7a100t FPGA 合成 clock rate 50 MHz 。

| Resource | percentage | number | total  |
| :------- | :--------- | :----- | :----- |
| LUT      | 30.30      | 19210  | 63400  |
| LUTRAM   | 5.79       | 1101   | 19000  |
| FF       | 12.14      | 15397  | 126800 |
| BRAM     | 39.26      | 53     | 135    |
| DSP      | 6.67       | 16     | 240    |
| IO       | 28.57      | 60     | 210    |
| BUFG     | 15.63      | 5      | 32     |
| MMCM     | 33.3       | 2      | 6      |
| PLL      | 16.67      | 1      | 6      |

功率： 1.018 W

# 測試結果

此專題進行期間，用以測試的 riscv-test 有經過一次更新，修改包含 mi 子集。此處以新版測試資料為主。虛擬記憶體的使用只針對 user mode 進行。

```
-----------------------------------------------
Testing: rv64mi
Testcase: rv64mi-p-sbreak                failed
Testcase: rv64mi-p-mcsr                  passed
Testcase: rv64mi-p-ma_addr               passed
Testcase: rv64mi-p-zicntr                passed
Testcase: rv64mi-p-scall                 passed
Testcase: rv64mi-p-csr                   failed
Testcase: rv64mi-p-lh-misaligned         passed
Testcase: rv64mi-p-breakpoint            failed
Testcase: rv64mi-p-instret_overflow      passed
Testcase: rv64mi-p-ma_fetch              failed
Testcase: rv64mi-p-ld-misaligned         passed
Testcase: rv64mi-p-sd-misaligned         passed
Testcase: rv64mi-p-sw-misaligned         passed
Testcase: rv64mi-p-pmpaddr               passed
Testcase: rv64mi-p-sh-misaligned         passed
Testcase: rv64mi-p-lw-misaligned         passed
Testcase: rv64mi-p-illegal               passed

Total case: 17 pass: 13 fail: 4
-----------------------------------------------
Testing: rv64si
Testcase: rv64si-p-sbreak                failed
Testcase: rv64si-p-dirty                 failed
Testcase: rv64si-p-scall                 passed
Testcase: rv64si-p-wfi                   passed
Testcase: rv64si-p-icache-alias          passed
Testcase: rv64si-p-ma_fetch              failed
Testcase: rv64si-p-csr                   passed

Total case: 7 pass: 4 fail: 3
-----------------------------------------------
Testing: rv64ui_p
Testcase: rv64ui-p-lbu                   passed
Testcase: rv64ui-p-slt                   passed
Testcase: rv64ui-p-sd                    passed
Testcase: rv64ui-p-srai                  passed
Testcase: rv64ui-p-srl                   passed
Testcase: rv64ui-p-ld_st                 passed
Testcase: rv64ui-p-addiw                 passed
Testcase: rv64ui-p-fence_i               failed
Testcase: rv64ui-p-sub                   passed
Testcase: rv64ui-p-lb                    passed
Testcase: rv64ui-p-simple                passed
Testcase: rv64ui-p-sltu                  passed
Testcase: rv64ui-p-slliw                 passed
Testcase: rv64ui-p-lhu                   passed
Testcase: rv64ui-p-lwu                   passed
Testcase: rv64ui-p-blt                   passed
Testcase: rv64ui-p-ori                   passed
Testcase: rv64ui-p-slti                  passed
Testcase: rv64ui-p-bge                   passed
Testcase: rv64ui-p-xori                  passed
Testcase: rv64ui-p-addi                  passed
Testcase: rv64ui-p-sh                    passed
Testcase: rv64ui-p-or                    passed
Testcase: rv64ui-p-sllw                  passed
Testcase: rv64ui-p-andi                  passed
Testcase: rv64ui-p-ld                    passed
Testcase: rv64ui-p-add                   passed
Testcase: rv64ui-p-bltu                  passed
Testcase: rv64ui-p-auipc                 passed
Testcase: rv64ui-p-bne                   passed
Testcase: rv64ui-p-addw                  passed
Testcase: rv64ui-p-xor                   passed
Testcase: rv64ui-p-srli                  passed
Testcase: rv64ui-p-sra                   passed
Testcase: rv64ui-p-beq                   passed
Testcase: rv64ui-p-bgeu                  passed
Testcase: rv64ui-p-subw                  passed
Testcase: rv64ui-p-st_ld                 passed
Testcase: rv64ui-p-sw                    passed
Testcase: rv64ui-p-sraiw                 passed
Testcase: rv64ui-p-sll                   passed
Testcase: rv64ui-p-jal                   passed
Testcase: rv64ui-p-sb                    passed
Testcase: rv64ui-p-and                   passed
Testcase: rv64ui-p-ma_data               failed
Testcase: rv64ui-p-jalr                  passed
Testcase: rv64ui-p-lw                    passed
Testcase: rv64ui-p-sltiu                 passed
Testcase: rv64ui-p-slli                  passed
Testcase: rv64ui-p-lui                   passed
Testcase: rv64ui-p-sraw                  passed
Testcase: rv64ui-p-srliw                 passed
Testcase: rv64ui-p-lh                    passed
Testcase: rv64ui-p-srlw                  passed

Total case: 54 pass: 52 fail: 2
-----------------------------------------------
Testing: rv64um_p
Testcase: rv64um-p-remw                  passed
Testcase: rv64um-p-mulhsu                passed
Testcase: rv64um-p-div                   passed
Testcase: rv64um-p-mulw                  passed
Testcase: rv64um-p-remuw                 passed
Testcase: rv64um-p-rem                   passed
Testcase: rv64um-p-mul                   passed
Testcase: rv64um-p-divw                  passed
Testcase: rv64um-p-remu                  passed
Testcase: rv64um-p-divuw                 passed
Testcase: rv64um-p-mulh                  passed
Testcase: rv64um-p-divu                  passed
Testcase: rv64um-p-mulhu                 passed

Total case: 13 pass: 13 fail: 0
-----------------------------------------------
Testing: rv64ua_p
Testcase: rv64ua-p-amoand_w              passed
Testcase: rv64ua-p-amomaxu_d             passed
Testcase: rv64ua-p-amoxor_w              passed
Testcase: rv64ua-p-amoadd_w              passed
Testcase: rv64ua-p-amomax_d              passed
Testcase: rv64ua-p-amoor_d               passed
Testcase: rv64ua-p-amoor_w               passed
Testcase: rv64ua-p-amominu_w             passed
Testcase: rv64ua-p-amomin_d              passed
Testcase: rv64ua-p-lrsc                  passed
Testcase: rv64ua-p-amoand_d              passed
Testcase: rv64ua-p-amominu_d             passed
Testcase: rv64ua-p-amoswap_w             passed
Testcase: rv64ua-p-amoswap_d             passed
Testcase: rv64ua-p-amoxor_d              passed
Testcase: rv64ua-p-amomaxu_w             passed
Testcase: rv64ua-p-amomax_w              passed
Testcase: rv64ua-p-amoadd_d              passed
Testcase: rv64ua-p-amomin_w              passed

Total case: 19 pass: 19 fail: 0
-----------------------------------------------
Testing: rv64ui_v
Testcase: rv64ui-v-bltu                  passed
Testcase: rv64ui-v-jalr                  passed
Testcase: rv64ui-v-sraiw                 passed
Testcase: rv64ui-v-auipc                 passed
Testcase: rv64ui-v-bne                   passed
Testcase: rv64ui-v-lh                    passed
Testcase: rv64ui-v-andi                  passed
Testcase: rv64ui-v-slli                  passed
Testcase: rv64ui-v-fence_i               passed
Testcase: rv64ui-v-ma_data               failed
Testcase: rv64ui-v-xori                  passed
Testcase: rv64ui-v-simple                passed
Testcase: rv64ui-v-slt                   passed
Testcase: rv64ui-v-lb                    passed
Testcase: rv64ui-v-add                   passed
Testcase: rv64ui-v-srliw                 passed
Testcase: rv64ui-v-sd                    passed
Testcase: rv64ui-v-lbu                   passed
Testcase: rv64ui-v-lui                   passed
Testcase: rv64ui-v-slti                  passed
Testcase: rv64ui-v-srlw                  passed
Testcase: rv64ui-v-jal                   passed
Testcase: rv64ui-v-xor                   passed
Testcase: rv64ui-v-sb                    passed
Testcase: rv64ui-v-lhu                   passed
Testcase: rv64ui-v-sra                   passed
Testcase: rv64ui-v-sw                    passed
Testcase: rv64ui-v-lwu                   passed
Testcase: rv64ui-v-blt                   passed
Testcase: rv64ui-v-ori                   passed
Testcase: rv64ui-v-st_ld                 passed
Testcase: rv64ui-v-srl                   passed
Testcase: rv64ui-v-lw                    passed
Testcase: rv64ui-v-srli                  passed
Testcase: rv64ui-v-sllw                  passed
Testcase: rv64ui-v-sraw                  passed
Testcase: rv64ui-v-sub                   passed
Testcase: rv64ui-v-bge                   passed
Testcase: rv64ui-v-sh                    passed
Testcase: rv64ui-v-sll                   passed
Testcase: rv64ui-v-beq                   passed
Testcase: rv64ui-v-slliw                 passed
Testcase: rv64ui-v-sltu                  passed
Testcase: rv64ui-v-ld_st                 passed
Testcase: rv64ui-v-and                   passed
Testcase: rv64ui-v-subw                  passed
Testcase: rv64ui-v-addw                  passed
Testcase: rv64ui-v-bgeu                  passed
Testcase: rv64ui-v-sltiu                 passed
Testcase: rv64ui-v-addiw                 passed
Testcase: rv64ui-v-or                    passed
Testcase: rv64ui-v-ld                    passed
Testcase: rv64ui-v-srai                  passed
Testcase: rv64ui-v-addi                  passed

Total case: 54 pass: 53 fail: 1
-----------------------------------------------
Testing: rv64um_v
Testcase: rv64um-v-mulh                  passed
Testcase: rv64um-v-remuw                 passed
Testcase: rv64um-v-divw                  passed
Testcase: rv64um-v-div                   passed
Testcase: rv64um-v-mulw                  passed
Testcase: rv64um-v-mulhu                 passed
Testcase: rv64um-v-rem                   passed
Testcase: rv64um-v-divu                  passed
Testcase: rv64um-v-mulhsu                passed
Testcase: rv64um-v-remw                  passed
Testcase: rv64um-v-mul                   passed
Testcase: rv64um-v-divuw                 passed
Testcase: rv64um-v-remu                  passed

Total case: 13 pass: 13 fail: 0
-----------------------------------------------
Testing: rv64ua_v
Testcase: rv64ua-v-amomaxu_d             passed
Testcase: rv64ua-v-amoor_w               passed
Testcase: rv64ua-v-amomaxu_w             passed
Testcase: rv64ua-v-amoswap_d             passed
Testcase: rv64ua-v-amoswap_w             passed
Testcase: rv64ua-v-amoand_d              passed
Testcase: rv64ua-v-amoor_d               passed
Testcase: rv64ua-v-amominu_w             passed
Testcase: rv64ua-v-amomin_d              passed
Testcase: rv64ua-v-amoxor_w              passed
Testcase: rv64ua-v-amoxor_d              passed
Testcase: rv64ua-v-amoadd_d              passed
Testcase: rv64ua-v-amominu_d             passed
Testcase: rv64ua-v-amoand_w              passed
Testcase: rv64ua-v-lrsc                  passed
Testcase: rv64ua-v-amomax_d              passed
Testcase: rv64ua-v-amomax_w              passed
Testcase: rv64ua-v-amoadd_w              passed
Testcase: rv64ua-v-amomin_w              passed

Total case: 19 pass: 19 fail: 0
```