/**
  ********************************************************************************************************************************************
  * @file    safety_core_macros.h 
  * @author  Myron Mychal
  * @version V2.0
  * @date    03-09-2020
  * @brief   Safety Core Module Macros
  * @note    Core macros for use with Shadowed RAM in Safety core
  *******************************************************************************************************************************************
  */

//
// Revision History
// 1.0 - MRM - Initial build
// 2.0 - MRM - repaired increment and decrement macros for public structure (was using the logical operations which are not defined for public structures)
// 2.1 - JJM - Renamed and Reorganized Macros, added extra documentation, added separate macros for assigning values to different data types

#ifndef _SAFETY_CORE_MACROS_H_
#define _SAFETY_CORE_MACROS_H_

// safety core firmware version in VV.XX.YY format
#define MEERKAT_FW_VERSION_MAJOR 0x3032 // VV // In ASCII
#define MEERKAT_FW_VERSION_MEDIAN 0x3030 // XX // In ASCII
#define MEERKAT_FW_VERSION_MINOR 0x3030 // YY // In ASCII


// module structure names

// The definition of the two macros below is dependent on the variable name associated with the Module's structure.
// It MUST match for all of the other macros to w
//			|     |
//			|     |
//			V     V
// Structure Pointers:
// - Reassign these in specific Module for use with the macros in this file
#define PUBLIC_STRUCT_POS  PPublicVars  // Assign Module Struct to Test Macros (Public Positive RAM)
#define PUBLIC_STRUCT_NEG  NPublicVars  // Assign Module Struct to Test Macros (Public Positive RAM)
#define PRIVATE_STRUCT_POS PPrivateVars // Assign Module Struct to Test Macros (Public Positive RAM)
#define PRIVATE_STRUCT_NEG NPrivateVars // Assign Module Struct to Test Macros (Public Positive RAM)
// - Get a value from the module's structure
#define GET_PUBLIC_MEMBER_POS(member_name)  (PUBLIC_STRUCT_POS.member_name)
#define GET_PUBLIC_MEMBER_NEG(member_name)  (PUBLIC_STRUCT_NEG.member_name)

#define GET_PRIVATE_MEMBER_POS(member_name) (PRIVATE_STRUCT_POS.member_name)
#define GET_PRIVATE_MEMBER_NEG(member_name) (PRIVATE_STRUCT_NEG.member_name)

// General Math Macros
#define GET_VALUE(invert_value, value)                                                                                           \
    ((invert_value) ? (value) : (~value)) // RAM_CHECK: if P = 1, use value,, if p = 0, the invert
#define SET_VALUE(data_object, new_value)     ((data_object) = (new_value)) // assigns a = b;
#define IS_GREATER(value_a, value_b)          ((value_a) > (value_b))       // is a > b?
#define IS_GREATOR_OR_EQUAL(value_a, value_b) ((value_a) >= (value_b))      // is a >= b?
// #define IS_LESS_THAN(value_a, value_b)                ((value_a) < (value_b))                           // is a < b?
#define IS_LESS_OR_EQUAL(value_a, value_b) ((value_a) <= (value_b)) // is a <= b?
// #define ARE_EQUAL(value_a, value_b)                   ((value_a) == (value_b))                          // is a == b?
// #define ARE_NOT_EQUAL(value_a, value_b)               ((value_a) != (value_b))                          // is a != b?
// #define LOGICAL_AND(value_a, value_b)                 ((value_a) && (value_b))                          // a bitwise AND with b
// #define COMMA(value_a, value_b)                       ((value_a), (value_b))
// #define FOO(value_a, value_b, value_m, value_n)                   (COMMA(SET_VALUE(a, m), SET_VALUE(b, n)))

// Local Variable - Value Assignment (PRAM/NRAM)
//   assign a value to a member of the module's structure
// - Public Structure
// -- RAM Check Macros: Set Value in both Positive and Negative RAM Structures
#define SET_PUBLIC_MEMBER_VALUE(member_name, new_value)                                                                          \
    (SET_VALUE(GET_PUBLIC_MEMBER_NEG(member_name), GET_VALUE(0, (new_value))));                                                  \
    (SET_VALUE(GET_PUBLIC_MEMBER_POS(member_name), GET_VALUE(1, (new_value))))
#define SET_PUBLIC_MEMBER_VALUE_U8(member_name, new_value)                                                                       \
    (SET_VALUE(GET_PUBLIC_MEMBER_NEG(member_name), (uint8_t) GET_VALUE(0, (new_value))));                                        \
    (SET_VALUE(GET_PUBLIC_MEMBER_POS(member_name), (uint8_t) GET_VALUE(1, (new_value))))
#define SET_PUBLIC_MEMBER_VALUE_U16(member_name, new_value)                                                                      \
    (SET_VALUE(GET_PUBLIC_MEMBER_NEG(member_name), (uint16_t) GET_VALUE(0, (new_value))));                                       \
    (SET_VALUE(GET_PUBLIC_MEMBER_POS(member_name), (uint16_t) GET_VALUE(1, (new_value))))
#define SET_PUBLIC_MEMBER_VALUE_U32(member_name, new_value)                                                                      \
    (SET_VALUE(GET_PUBLIC_MEMBER_NEG(member_name), (uint32_t) GET_VALUE(0, (new_value))));                                       \
    (SET_VALUE(GET_PUBLIC_MEMBER_POS(member_name), (uint32_t) GET_VALUE(1, (new_value))))
// --- For Initializing Loops
// ---- this is similar to 'SET_PUBLIC_MEMBER_VALUE', but operations are comma-separated (not semi-colon)
// ---- this keeps the 'semicolon' from interfering with the for() statement's expectation of semicolon's
#define SET_PUBLIC_MEMBER_VALUE_IN_LOOP(member_name, value)                                                                      \
    (SET_VALUE(GET_PUBLIC_MEMBER_NEG(member_name), GET_VALUE(0, (value)))),                                                      \
        (SET_VALUE(GET_PUBLIC_MEMBER_POS(member_name), GET_VALUE(1, (value))))
// -- Basic Arithmetic in both RAM Structures (Public)
// --- Add
#define ADD_TO_PUBLIC_MEMBER_VALUE(member_name, x)                                                                               \
    (SET_PUBLIC_MEMBER_VALUE_IN_LOOP(member_name, (GET_PUBLIC_MEMBER_POS(member_name) + x)))
#define INC_PUBLIC_MEMBER_VALUE(member_name) (ADD_TO_PUBLIC_MEMBER_VALUE(member_name, 1))
// --- Subtract
#define SUB_FROM_PUBLIC_MEMBER_VALUE(member_name, x)                                                                             \
    (SET_PUBLIC_MEMBER_VALUE_IN_LOOP(member_name, (GET_PUBLIC_MEMBER_POS(member_name) - x)))
#define DEC_PUBLIC_MEMBER_VALUE(member_name) (SUB_FROM_PUBLIC_MEMBER_VALUE(member_name, 1))
// --- Boolean
#define PRIVATE_MEMBER_VALUE_IS_GREATER_OR_EQUAL(member_name, value)                                                             \
    (IS_GREATOR_OR_EQUAL(GET_PRIVATE_MEMBER_POS(member_name), (value)))
#define PRIVATE_MEMBER_VALUE_IS_LESS_OR_EQUAL(member_name, value) (IS_LESS_OR_EQUAL(GET_PRIVATE_MEMBER_POS(member_name), (value)))

// - Private RAM Structure Macros
// -- RAM Check Macros: Set Value in both Positive and Negative RAM Structures
// -- Note on "Self Assignment" (Positive RAM)
// --- PRIVATE_STRUCT_POS.blockAddrPtr += 4;
// --- is equivalent to SET_PRIVATE_MEMBER_VALUE_U32(blockAddrPtr, GET_PRIVATE_MEMBER_POS(blockAddrPtr) + 4);
// --- to allow self assignments to items that read from Positive RAM,
// --- 'SET_VALUE'ing to negative RAM must occur before EQUATING to positive RAM
// --- or the math operation happens twice in the negative space
// -- Unsigned Integers
#define SET_PRIVATE_MEMBER_VALUE(member_name, value)                                                                             \
    (SET_VALUE(GET_PRIVATE_MEMBER_NEG(member_name), GET_VALUE(0, (value))));                                                     \
    (SET_VALUE(GET_PRIVATE_MEMBER_POS(member_name), GET_VALUE(1, (value))))
#define SET_PRIVATE_MEMBER_VALUE_U8(member_name, value)                                                                          \
    (SET_VALUE(GET_PRIVATE_MEMBER_NEG(member_name), (uint8_t) GET_VALUE(0, (value))));                                           \
    (SET_VALUE(GET_PRIVATE_MEMBER_POS(member_name), (uint8_t) GET_VALUE(1, (value))))
#define SET_PRIVATE_MEMBER_VALUE_U16(member_name, value)                                                                         \
    (SET_VALUE(GET_PRIVATE_MEMBER_NEG(member_name), (uint16_t) GET_VALUE(0, (value))));                                          \
    (SET_VALUE(GET_PRIVATE_MEMBER_POS(member_name), (uint16_t) GET_VALUE(1, (value))))
#define SET_PRIVATE_MEMBER_VALUE_U32(member_name, value)                                                                         \
    (SET_VALUE(GET_PRIVATE_MEMBER_NEG(member_name), (uint32_t) GET_VALUE(0, ((uint32_t) value))));                               \
    (SET_VALUE(GET_PRIVATE_MEMBER_POS(member_name), (uint32_t) GET_VALUE(1, ((uint32_t) value))))
// -- Signed Integers
#define SET_PRIVATE_MEMBER_VALUE_I16(member_name, value)                                                                         \
    (SET_VALUE(GET_PRIVATE_MEMBER_NEG(member_name), (int16_t) GET_VALUE(0, (value))));                                          \
    (SET_VALUE(GET_PRIVATE_MEMBER_POS(member_name), (int16_t) GET_VALUE(1, (value))))
        
// -- Exception: to allow self assignments to items that read from Negative RAM
// --- 'SET_VALUE'ing to positive RAM must occur before EQUATING to negataive RAM
// ---- or the math operation happens twice in the positive space
// ----- this was implemented for a case where RAM Checker is checking it's own RAM objects
#define N_SET_PRIVATE_MEMBER_VALUE_U8(member_name, value)                                                                        \
    (SET_VALUE(GET_PRIVATE_MEMBER_POS(member_name), (uint8_t) GET_VALUE(1, (value))));                                           \
    (SET_VALUE(GET_PRIVATE_MEMBER_NEG(member_name), (uint8_t) GET_VALUE(0, (value))))
// --- For Initializing Loops
// ---- this is similar to 'SET_PRIVATE_MEMBER_VALUE', but operations are comma-separated (not semi-colon)
// ---- this keeps the 'semicolon' from interfering with the for() statement's expectation of semicolons
#define SET_PRIVATE_MEMBER_VALUE_IN_LOOP(member_name, value)                                                                     \
    (SET_VALUE(GET_PRIVATE_MEMBER_NEG(member_name), GET_VALUE((0), (value)))),                                                   \
        (SET_VALUE(GET_PRIVATE_MEMBER_POS(member_name), GET_VALUE((1), (value))))

// -- Basic Arithmetic in both RAM Structures (Private)
// --- Add
// REVIEW: Simplify ADD/INC/SUB/DEC Macros? if possible
// - We seem to have an issue where these need to be evaluated immediately
// -- mainly so that they can be used in 'for loops'
// ----  the '?' forces evaluation, simplifying efforts so far, lead to compiler errors 
#define ADD_TO_PRIVATE_MEMBER_VALUE(member_name, value)                                                                          \
    ((PRIVATE_MEMBER_VALUE_IS_LESS_OR_EQUAL(member_name, value)) ?                                                               \
         (SET_PRIVATE_MEMBER_VALUE_IN_LOOP(member_name, (GET_PRIVATE_MEMBER_POS(member_name) + value))) :                        \
         (SET_PRIVATE_MEMBER_VALUE_IN_LOOP(member_name, (GET_PRIVATE_MEMBER_POS(member_name) + value))))
#define INC_PRIVATE_MEMBER_VALUE(member_name) (ADD_TO_PRIVATE_MEMBER_VALUE(member_name, 1))
// --- Subtract
#define SUB_FROM_PRIVATE_MEMBER_VALUE(member_name, value)                                                                        \
    ((PRIVATE_MEMBER_VALUE_IS_GREATER_OR_EQUAL(member_name, value)) ?                                                            \
         (SET_PRIVATE_MEMBER_VALUE_IN_LOOP(member_name, (GET_PRIVATE_MEMBER_POS(member_name) - value))) :                        \
         (SET_PRIVATE_MEMBER_VALUE_IN_LOOP(member_name, (GET_PRIVATE_MEMBER_POS(member_name) + 1 - value))))
#define DEC_PRIVATE_MEMBER_VALUE(member_name) (SUB_FROM_PRIVATE_MEMBER_VALUE(member_name, 1))

#endif
