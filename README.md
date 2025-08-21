# Senior Project

National Yang Ming Chiao Tung University, Department of Computer Science, Computer Science and Engineering Projects.

A 64-bit version of a RISC-V processor Aquila. For the original version of Aquila, please reference to: [https://github.com/eisl-nctu/aquila](https://github.com/eisl-nctu/aquila)

## Specification

The original Aquila core supports rv32ima_zicsr_zifencei, while the 64-bit version supports rv64ima_zicsr_zifencei.

## Hardware

All HDL code is stored in the "Hardware" directory and is divided into three versions.

1.  tcm_verison
2. dram_version
3.  mmu_version

The three versions have corresponding files that describe the changes compared to 32-bit processors.

## Software

All software code is stored in the "Software" directory. This primarily includes Aquila's original C library and bootloader, as well as some test programs. 

A file called "note_sw.md" documents the changes compared to the 32-bit processor.

---

# 專題

國立陽明交通大學資訊工程學系，「資訊工程專題」成果， RISC-V 處理器 Aquila 的 64 位元版本。關於原本的 Aquila 處理器，請參考：[https://github.com/eisl-nctu/aquila](https://github.com/eisl-nctu/aquila)

## 規格

原本的 Aquila 處理器支援 rv32ima_zicsr_zifencei 的 ISA 。 64 位元版本支援 rv64ima_zicsr_zifencei 。

## 硬體

「Hardware」資料夾存放所有 HDL 程式碼。分為三個版本：

1.  tcm_version
2. dram_version
3.  mmu_version

三個版本中有對應的文件說明和 32 位元處理器相比的改動內容。

## 軟體

「Software」資料夾存放所有軟體程式碼。主要包含 Aquila 原有的 C 函式庫和 bootloader ，以及一些用以測試的程式。一份 「note_sw.md」 的檔案紀錄和 32 位元處理器相比的改動內容。