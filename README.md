
# Sequential Implementation of AES-128 on FPGA

## Introduction

This project focuses on understanding the specifications of AES-128 and implementing the encryption (and key scheduling) algorithm on an FPGA. The goal is to leverage the FPGA as a hardware accelerator to enhance the speed of the encryption process. Unlike programmatic implementation on an MCU, which can be slower, a hardware approach is more viable. A significant challenge encountered was the limitations of our FPGA. Due to its size, it wasn't feasible to implement all of the rounds in one large block of combinational logic. Instead, a sequential process was designed to fit the architecture on the FPGA.

## Design and Testing Methodology

The design began with creating modules and testbenches for each of the "helper" modules, including Add Round Key, Rot Word, Shift Rows, Sub Bytes, and Sub Word. These modules play a crucial role during key expansion and each encryption round. Designing and testing each module individually ensured a strong foundation for the overall encryption logic.

The core architecture is based on a large Finite State Machine (FSM) that performs both key expansion and the encryption. The FSM meticulously tracks the round, updates the expanded key, and the ciphertext until the completion of round 10, signaling the end of the encryption process. A testbench based on AES test vectors was designed to validate this FSM, which was then utilized in the aes_core module. The aes_core module, a smaller FSM, manages the encryption of new plaintext messages with new keys. Adjustments included adding delays at the start to ensure proper population of plaintext and keys before encryption. These modifications ensured successful operation of both the aes_core and aes_spi testbenches.

## Results and Discussion

This project enabled a deep dive into AES encryption, particularly key expansion. The designed architecture was successfully implemented on the FPGA, utilizing 1337 out of the 5280 slice registers and 2443 out of the 5280 4 input Look Up Tables. This achievement demonstrated the capability to simulate AES encryption with given keys and plaintext, along with core and spi modules for MCU interaction.

## Conclusions

This project has been immensely fulfilling, merging interests in security, cryptography, and hardware. Initial attempts at implementing AES on an FPGA during the summer faced challenges due to limited FPGA experience. However, this endeavor provided valuable insights, especially the utility of an AES visualization website for understanding key expansion beyond the pseudocode. The project required approximately 12 hours to complete, with significant time devoted to conceptualizing and diagramming the FSM for key expansion and encryption. The final design meets all set proficiency requirements, marking a successful implementation of AES-128 on an FPGA.
