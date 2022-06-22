;******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
; File Name          : stm32g0xx_STLcpurunIAR.s
; Author             : MCD Application Team
; Date First Issued  : 28-Jun-2019
; Version            : V2.3.0
; Description        : This file contains the Cortex-M3 CPU tests to be done
;                      during run-time.
;*******************************************************************************
; @attention
;
; <h2><center>&copy; COPYRIGHT(c) 2019 STMicroelectronics</center></h2>
;
; Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
; You may not use this file except in compliance with the License.
; You may obtain a copy of the License at:
;
;        http://www.st.com/software_license_agreement_liberty_v2
;
; Unless required by applicable law or agreed to in writing, software
; distributed under the License is distributed on an "AS IS" BASIS,
; WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
; See the License for the specific language governing permissions and
; limitations under the License.
;
;******************************************************************************

  SECTION myConstantData:CONST(2)

conAA       DCD     0xAAAAAAAA
con55       DCD     0x55555555

  SECTION .text:CODE(2)

  ; Reference to the FailSafe routine to be executed in case of non-recoverable
  ; failure
  EXTERN RegCheckFail

    ; C variables for control flow monitoring
  EXTERN CtrlFlowCnt
  EXTERN CtrlFlowCntInv
  //EXTERN conAA
  //EXTERN con55

;*******************************************************************************
; Function Name  : STL_RunTimeCPUTest
; Description    : Cortex-M0 CPU test during run-time
;                  Note: when possible, BRANCH are 16-bit only (depending on
;                  relative offset to final BL instruction)
; Input          : None.
; Output         : Branch directly to a Fail Safe routine in case of failure
; Return         : CPUTEST_SUCCESS (=1) if test is ok
;*******************************************************************************/

//STL_RunTimeCPURegR0_R3:
    //EXPORT STL_RunTimeCPURegR0_R3

STL_RunTimeCPURegCheck:
    EXPORT STL_RunTimeCPURegCheck    
    
    PUSH {R4}              ; Safe registers
    ; This is for control flow test (ENTRY point)
    //LDR R0,=CtrlFlowCnt
    ; Assumes R1 OK; If not, error will be detected by R1 test and Ctrl flow test later on
    //LDR R1,[R0]
    //ADDS R1,R1,#0x3	 ; CtrlFlowCnt += OxO3
    //STR R1,[R0]


    ; Register R1
    LDR R0, =conAA
    LDR R1,[R0]
    LDR R0,[R0]
    CMP R0,R1
    BNE CPUTestFail
    LDR R0, =con55
    LDR R1,[R0]
    LDR R0,[R0]
    CMP R0,R1
    BNE CPUTestFail

    ; Register R2
    LDR R0, =conAA
    LDR R2,[R0]
    LDR R0,[R0]
    CMP R0,R2
    BNE CPUTestFail
    LDR R0, =con55
    LDR R2,[R0]
    LDR R0,[R0]
    CMP R0,R2
    BNE CPUTestFail

    ; Register R3
    LDR R0, =conAA
    LDR R3,[R0]
    LDR R0,[R0]
    CMP R0,R3
    BNE CPUTestFail
    LDR R0, =con55
    LDR R3,[R0]
    LDR R0,[R0]
    CMP R0,R3
    BNE CPUTestFail
    
    ; Register R4
    LDR R0, =conAA
    LDR R4,[R0] 
    LDR R0,[R0]
    CMP R0,R4
    BNE CPUTestFail
    LDR R0, =con55
    LDR R4,[R0]
    LDR R0,[R0]
    CMP R0,R4
    BNE CPUTestFail
    POP {R4}              ; Restore registers

    PUSH {R5-R8}          ; Save registers
   ; Register R5
    LDR R0, =conAA
    LDR R5,[R0]
    LDR R0,[R0]
    CMP R0,R5
    BNE CPUTestFail5_8
    LDR R0, =con55
    LDR R5,[R0]
    LDR R0,[R0]
    CMP R0,R5
    BNE CPUTestFail5_8

    ; Register R6
    LDR R0, =conAA
    LDR R6,[R0]
    LDR R0,[R0]
    CMP R0,R6
    BNE CPUTestFail5_8
    LDR R0, =con55
    LDR R6,[R0]
    LDR R0,[R0]
    CMP R0,R6
    BNE CPUTestFail5_8

    ; Register R7
    LDR R0, =conAA
    LDR R7,[R0]
    LDR R0,[R0]
    CMP R0,R7
    BNE CPUTestFail5_8
    LDR R0, =con55
    LDR R7,[R0]
    LDR R0,[R0]
    CMP R0,R7
    BNE CPUTestFail5_8

    ; Register R8
    LDR R0, =conAA
    LDR R0,[R0]
    MOV R8,R0
    CMP R0,R8
    BNE CPUTestFail5_8
    LDR R0, =con55
    LDR R0,[R0]
    MOV R8,R0
    CMP R0,R8
    BNE CPUTestFail5_8
    POP {R5-R8}              ; Restore registers

    PUSH {R9-R12}            ; Save registers

    ; Register R9
    LDR R0, =conAA
    LDR R0,[R0]
    MOV R9,R0
    CMP R0,R9
    BNE CPUTestFail9_12
    LDR R0, =con55
    LDR R0,[R0]
    MOV R9,R0
    CMP R0,R9
    BNE CPUTestFail9_12

    ; Register R10
    LDR R0, =conAA
    LDR R0,[R0]
    MOV R10,R0
    CMP R0,R10
    BNE CPUTestFail9_12
    LDR R0, =con55
    LDR R0,[R0]
    MOV R10,R0
    CMP R0,R10
    BNE CPUTestFail9_12

    ; Register R11
    LDR R0, =conAA
    LDR R0,[R0]
    MOV R11,R0
    CMP R0,R11
    BNE CPUTestFail9_12
    LDR R0, =con55
    LDR R0,[R0]
    MOV R11,R0
    CMP R0,R11
    BNE CPUTestFail9_12

    ; Register R12
    LDR R0, =conAA
    LDR R0,[R0]
    MOV R12,R0
    CMP R0,R12
    BNE CPUTestFail9_12
    LDR R0, =con55
    LDR R0,[R0]
    MOV R12,R0
    CMP R0,R12
    BNE CPUTestFail9_12

    //LDR R0,=CtrlFlowCntInv
    //LDR R1,[R0]
    //SUBS R1,R1,#0x3	 ; CtrlFlowCntInv -= OxO3
    //STR R1,[R0]

    POP {R9-R12}              ; Restore registers
    //BX LR               ; return to the caller

    ; Link register R14	cannot be tested an error should be detected by	Ctrl flow test later

// Register Pattern Check

    PUSH {R4-R7}              ; Safe registers
    ; Ramp pattern verification	(R0 is not tested)
    MOVS R1, #0x1             ; For ramp test
    CMP R1, #0x01
    BNE CPUTestFail4_7
    MOVS R2, #0x2             ; For ramp test
    CMP R2, #0x02
    BNE CPUTestFail4_7
    MOVS R3, #0x3             ; For ramp test
    CMP R3, #0x03
    BNE CPUTestFail4_7
    MOVS R4, #0x4             ; For ramp test
    CMP R4, #0x04
    BNE CPUTestFail4_7
    MOVS R5, #0x5             ; For ramp test
    CMP R5, #0x05
    BNE CPUTestFail4_7
    MOVS R6, #0x6             ; For ramp test
    CMP R6, #0x06
    BNE CPUTestFail4_7
    MOVS R7, #0x7             ; For ramp test
    CMP R7, #0x07
    BNE CPUTestFail4_7    
    POP {R4-R7}              ; Restore registers
    
    PUSH {R8-R12}              ; Safe registers
    MOVS R8, #0x8             ; For ramp test
    MOVS R0, #0x08
    CMP R0,R8
    BNE CPUTestFail8_12
    MOVS R9, #0x9             ; For ramp test
    MOVS R0, #0x09
    CMP R0,R9
    BNE CPUTestFail8_12
    MOVS R10, #0xA             ; For ramp test
    MOVS R0, #0x0A
    CMP R0,R10
    MOVS R11, #0xB             ; For ramp test
    MOVS R0, #0x0B
    CMP R0,R11
    BNE CPUTestFail8_12
    MOVS R12, #0xC             ; For ramp test
    MOVS R0, #0x0C
    CMP R0,R12
    BNE CPUTestFail8_12
    POP {R8-R12}              ; Restore registers
    ; Control flow test (EXIT point)
    //LDR R0,=CtrlFlowCntInv
    //LDR R1,[R0]
    //SUBS R1,R1,#0x3	 ; CtrlFlowCntInv -= OxO3
    //STR R1,[R0]

    //MOVS R0, #0x1       ; CPUTEST_SUCCESS
    BX LR               ; return to the caller

CPUTestFail4_7
    POP {R4-R7}         ; Restore registers
    PUSH {LR}           ; Preserve the return function address
    BL RegCheckFail
    POP {LR}            ; Restore the return function address
    BX LR               ; return to the caller

CPUTestFail5_8
    POP {R5-R8}         ; Restore registers
    PUSH {LR}           ; Preserve the return function address
    BL RegCheckFail 
    POP {LR}            ; Restore the return function address
    BX LR               ; return to the caller
    
CPUTestFail8_12
    POP {R8-R12}        ; Restore registers
    PUSH {LR}           ; Preserve the return function address
    BL RegCheckFail
    POP {LR}            ; Restore the return function address
    BX LR               ; return to the caller
CPUTestFail9_12
    POP {R9-R12}        ; Restore registers
    PUSH {LR}           ; Preserve the return function address
    BL RegCheckFail
    POP {LR}            ; Restore the return function address
    BX LR               ; return to the caller
CPUTestFail
    POP {R4}            ; Restore registers*/
    PUSH {LR}           ; Preserve the return function address
    BL RegCheckFail
    POP {LR}            ; Restore the return function address
    BX LR               ; return to the caller
  END

;******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE*****
