// Verilated -*- C++ -*-
// DESCRIPTION: Verilator output: Tracing implementation internals
#include "verilated_vcd_c.h"
#include "Vnewton_raphson_divider__Syms.h"


VL_ATTR_COLD void Vnewton_raphson_divider___024root__trace_init_sub__TOP__0(Vnewton_raphson_divider___024root* vlSelf, VerilatedVcd* tracep) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vnewton_raphson_divider___024root__trace_init_sub__TOP__0\n"); );
    Vnewton_raphson_divider__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Init
    const int c = vlSymsp->__Vm_baseCode;
    // Body
    tracep->pushPrefix("$rootio", VerilatedTracePrefixType::SCOPE_MODULE);
    tracep->declBit(c+10,0,"clk",-1, VerilatedTraceSigDirection::INPUT, VerilatedTraceSigKind::WIRE, VerilatedTraceSigType::LOGIC, false,-1);
    tracep->declBit(c+11,0,"nRST",-1, VerilatedTraceSigDirection::INPUT, VerilatedTraceSigKind::WIRE, VerilatedTraceSigType::LOGIC, false,-1);
    tracep->declBit(c+12,0,"request",-1, VerilatedTraceSigDirection::INPUT, VerilatedTraceSigKind::WIRE, VerilatedTraceSigType::LOGIC, false,-1);
    tracep->declBus(c+13,0,"divider",-1, VerilatedTraceSigDirection::INPUT, VerilatedTraceSigKind::WIRE, VerilatedTraceSigType::LOGIC, false,-1, 31,0);
    tracep->declBus(c+14,0,"dividend",-1, VerilatedTraceSigDirection::INPUT, VerilatedTraceSigKind::WIRE, VerilatedTraceSigType::LOGIC, false,-1, 31,0);
    tracep->declBus(c+15,0,"quotient",-1, VerilatedTraceSigDirection::OUTPUT, VerilatedTraceSigKind::WIRE, VerilatedTraceSigType::LOGIC, false,-1, 31,0);
    tracep->declBit(c+16,0,"ready",-1, VerilatedTraceSigDirection::OUTPUT, VerilatedTraceSigKind::WIRE, VerilatedTraceSigType::LOGIC, false,-1);
    tracep->popPrefix();
    tracep->pushPrefix("newton_raphson_divider", VerilatedTracePrefixType::SCOPE_MODULE);
    tracep->declBus(c+17,0,"FRACTIONAL_BITS",-1, VerilatedTraceSigDirection::NONE, VerilatedTraceSigKind::PARAMETER, VerilatedTraceSigType::LOGIC, false,-1, 31,0);
    tracep->declBus(c+18,0,"ITERATIONS",-1, VerilatedTraceSigDirection::NONE, VerilatedTraceSigKind::PARAMETER, VerilatedTraceSigType::LOGIC, false,-1, 31,0);
    tracep->declBit(c+10,0,"clk",-1, VerilatedTraceSigDirection::INPUT, VerilatedTraceSigKind::WIRE, VerilatedTraceSigType::LOGIC, false,-1);
    tracep->declBit(c+11,0,"nRST",-1, VerilatedTraceSigDirection::INPUT, VerilatedTraceSigKind::WIRE, VerilatedTraceSigType::LOGIC, false,-1);
    tracep->declBit(c+12,0,"request",-1, VerilatedTraceSigDirection::INPUT, VerilatedTraceSigKind::WIRE, VerilatedTraceSigType::LOGIC, false,-1);
    tracep->declBus(c+13,0,"divider",-1, VerilatedTraceSigDirection::INPUT, VerilatedTraceSigKind::WIRE, VerilatedTraceSigType::LOGIC, false,-1, 31,0);
    tracep->declBus(c+14,0,"dividend",-1, VerilatedTraceSigDirection::INPUT, VerilatedTraceSigKind::WIRE, VerilatedTraceSigType::LOGIC, false,-1, 31,0);
    tracep->declBus(c+15,0,"quotient",-1, VerilatedTraceSigDirection::OUTPUT, VerilatedTraceSigKind::WIRE, VerilatedTraceSigType::LOGIC, false,-1, 31,0);
    tracep->declBit(c+16,0,"ready",-1, VerilatedTraceSigDirection::OUTPUT, VerilatedTraceSigKind::WIRE, VerilatedTraceSigType::LOGIC, false,-1);
    tracep->declBus(c+1,0,"i_divider",-1, VerilatedTraceSigDirection::NONE, VerilatedTraceSigKind::VAR, VerilatedTraceSigType::LOGIC, false,-1, 31,0);
    tracep->declBus(c+2,0,"i_dividend",-1, VerilatedTraceSigDirection::NONE, VerilatedTraceSigKind::VAR, VerilatedTraceSigType::LOGIC, false,-1, 31,0);
    tracep->declQuad(c+3,0,"i_quotient",-1, VerilatedTraceSigDirection::NONE, VerilatedTraceSigKind::VAR, VerilatedTraceSigType::LOGIC, false,-1, 62,0);
    tracep->declBit(c+19,0,"negative",-1, VerilatedTraceSigDirection::NONE, VerilatedTraceSigKind::VAR, VerilatedTraceSigType::LOGIC, false,-1);
    tracep->declBus(c+5,0,"shift_count",-1, VerilatedTraceSigDirection::NONE, VerilatedTraceSigKind::VAR, VerilatedTraceSigType::LOGIC, false,-1, 4,0);
    tracep->declBus(c+6,0,"iter_count",-1, VerilatedTraceSigDirection::NONE, VerilatedTraceSigKind::VAR, VerilatedTraceSigType::LOGIC, false,-1, 4,0);
    tracep->declBus(c+7,0,"shift_dir",-1, VerilatedTraceSigDirection::NONE, VerilatedTraceSigKind::VAR, VerilatedTraceSigType::LOGIC, false,-1, 1,0);
    tracep->declBit(c+8,0,"fault",-1, VerilatedTraceSigDirection::NONE, VerilatedTraceSigKind::VAR, VerilatedTraceSigType::LOGIC, false,-1);
    tracep->declBus(c+20,0,"INITIAL_GUESS",-1, VerilatedTraceSigDirection::NONE, VerilatedTraceSigKind::PARAMETER, VerilatedTraceSigType::LOGIC, false,-1, 31,0);
    tracep->declBus(c+21,0,"ONE",-1, VerilatedTraceSigDirection::NONE, VerilatedTraceSigKind::PARAMETER, VerilatedTraceSigType::LOGIC, false,-1, 31,0);
    tracep->declBus(c+22,0,"TWO",-1, VerilatedTraceSigDirection::NONE, VerilatedTraceSigKind::PARAMETER, VerilatedTraceSigType::LOGIC, false,-1, 31,0);
    tracep->declBus(c+23,0,"FOUR",-1, VerilatedTraceSigDirection::NONE, VerilatedTraceSigKind::PARAMETER, VerilatedTraceSigType::LOGIC, false,-1, 31,0);
    tracep->declBus(c+24,0,"ONE_HALF",-1, VerilatedTraceSigDirection::NONE, VerilatedTraceSigKind::PARAMETER, VerilatedTraceSigType::LOGIC, false,-1, 31,0);
    tracep->declBus(c+25,0,"ONE_FOURTH",-1, VerilatedTraceSigDirection::NONE, VerilatedTraceSigKind::PARAMETER, VerilatedTraceSigType::LOGIC, false,-1, 31,0);
    tracep->declBus(c+26,0,"ONE_EIGHTH",-1, VerilatedTraceSigDirection::NONE, VerilatedTraceSigKind::PARAMETER, VerilatedTraceSigType::LOGIC, false,-1, 31,0);
    tracep->declBus(c+27,0,"LEFT_SHIFT",-1, VerilatedTraceSigDirection::NONE, VerilatedTraceSigKind::PARAMETER, VerilatedTraceSigType::LOGIC, false,-1, 1,0);
    tracep->declBus(c+28,0,"RIGHT_SHIFT",-1, VerilatedTraceSigDirection::NONE, VerilatedTraceSigKind::PARAMETER, VerilatedTraceSigType::LOGIC, false,-1, 1,0);
    tracep->declBus(c+29,0,"STATE_IDLE",-1, VerilatedTraceSigDirection::NONE, VerilatedTraceSigKind::PARAMETER, VerilatedTraceSigType::LOGIC, false,-1, 3,0);
    tracep->declBus(c+30,0,"STATE_SHIFT_IN_RANGE",-1, VerilatedTraceSigDirection::NONE, VerilatedTraceSigKind::PARAMETER, VerilatedTraceSigType::LOGIC, false,-1, 3,0);
    tracep->declBus(c+31,0,"STATE_NEWTON_ITER",-1, VerilatedTraceSigDirection::NONE, VerilatedTraceSigKind::PARAMETER, VerilatedTraceSigType::LOGIC, false,-1, 3,0);
    tracep->declBus(c+32,0,"STATE_CALCULATE_QUOTIENT",-1, VerilatedTraceSigDirection::NONE, VerilatedTraceSigKind::PARAMETER, VerilatedTraceSigType::LOGIC, false,-1, 3,0);
    tracep->declBus(c+9,0,"state",-1, VerilatedTraceSigDirection::NONE, VerilatedTraceSigKind::VAR, VerilatedTraceSigType::LOGIC, false,-1, 3,0);
    tracep->popPrefix();
}

VL_ATTR_COLD void Vnewton_raphson_divider___024root__trace_init_top(Vnewton_raphson_divider___024root* vlSelf, VerilatedVcd* tracep) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vnewton_raphson_divider___024root__trace_init_top\n"); );
    Vnewton_raphson_divider__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    Vnewton_raphson_divider___024root__trace_init_sub__TOP__0(vlSelf, tracep);
}

VL_ATTR_COLD void Vnewton_raphson_divider___024root__trace_const_0(void* voidSelf, VerilatedVcd::Buffer* bufp);
VL_ATTR_COLD void Vnewton_raphson_divider___024root__trace_full_0(void* voidSelf, VerilatedVcd::Buffer* bufp);
void Vnewton_raphson_divider___024root__trace_chg_0(void* voidSelf, VerilatedVcd::Buffer* bufp);
void Vnewton_raphson_divider___024root__trace_cleanup(void* voidSelf, VerilatedVcd* /*unused*/);

VL_ATTR_COLD void Vnewton_raphson_divider___024root__trace_register(Vnewton_raphson_divider___024root* vlSelf, VerilatedVcd* tracep) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vnewton_raphson_divider___024root__trace_register\n"); );
    Vnewton_raphson_divider__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    tracep->addConstCb(&Vnewton_raphson_divider___024root__trace_const_0, 0U, vlSelf);
    tracep->addFullCb(&Vnewton_raphson_divider___024root__trace_full_0, 0U, vlSelf);
    tracep->addChgCb(&Vnewton_raphson_divider___024root__trace_chg_0, 0U, vlSelf);
    tracep->addCleanupCb(&Vnewton_raphson_divider___024root__trace_cleanup, vlSelf);
}

VL_ATTR_COLD void Vnewton_raphson_divider___024root__trace_const_0_sub_0(Vnewton_raphson_divider___024root* vlSelf, VerilatedVcd::Buffer* bufp);

VL_ATTR_COLD void Vnewton_raphson_divider___024root__trace_const_0(void* voidSelf, VerilatedVcd::Buffer* bufp) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vnewton_raphson_divider___024root__trace_const_0\n"); );
    // Init
    Vnewton_raphson_divider___024root* const __restrict vlSelf VL_ATTR_UNUSED = static_cast<Vnewton_raphson_divider___024root*>(voidSelf);
    Vnewton_raphson_divider__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    // Body
    Vnewton_raphson_divider___024root__trace_const_0_sub_0((&vlSymsp->TOP), bufp);
}

VL_ATTR_COLD void Vnewton_raphson_divider___024root__trace_const_0_sub_0(Vnewton_raphson_divider___024root* vlSelf, VerilatedVcd::Buffer* bufp) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vnewton_raphson_divider___024root__trace_const_0_sub_0\n"); );
    Vnewton_raphson_divider__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Init
    uint32_t* const oldp VL_ATTR_UNUSED = bufp->oldp(vlSymsp->__Vm_baseCode);
    // Body
    bufp->fullIData(oldp+17,(0x10U),32);
    bufp->fullIData(oldp+18,(4U),32);
    bufp->fullBit(oldp+19,(vlSelfRef.newton_raphson_divider__DOT__negative));
    bufp->fullIData(oldp+20,(0x1c000U),32);
    bufp->fullIData(oldp+21,(0x10000U),32);
    bufp->fullIData(oldp+22,(0x20000U),32);
    bufp->fullIData(oldp+23,(0x40000U),32);
    bufp->fullIData(oldp+24,(0x8000U),32);
    bufp->fullIData(oldp+25,(0x4000U),32);
    bufp->fullIData(oldp+26,(0x2000U),32);
    bufp->fullCData(oldp+27,(0U),2);
    bufp->fullCData(oldp+28,(1U),2);
    bufp->fullCData(oldp+29,(0U),4);
    bufp->fullCData(oldp+30,(1U),4);
    bufp->fullCData(oldp+31,(2U),4);
    bufp->fullCData(oldp+32,(3U),4);
}

VL_ATTR_COLD void Vnewton_raphson_divider___024root__trace_full_0_sub_0(Vnewton_raphson_divider___024root* vlSelf, VerilatedVcd::Buffer* bufp);

VL_ATTR_COLD void Vnewton_raphson_divider___024root__trace_full_0(void* voidSelf, VerilatedVcd::Buffer* bufp) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vnewton_raphson_divider___024root__trace_full_0\n"); );
    // Init
    Vnewton_raphson_divider___024root* const __restrict vlSelf VL_ATTR_UNUSED = static_cast<Vnewton_raphson_divider___024root*>(voidSelf);
    Vnewton_raphson_divider__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    // Body
    Vnewton_raphson_divider___024root__trace_full_0_sub_0((&vlSymsp->TOP), bufp);
}

VL_ATTR_COLD void Vnewton_raphson_divider___024root__trace_full_0_sub_0(Vnewton_raphson_divider___024root* vlSelf, VerilatedVcd::Buffer* bufp) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vnewton_raphson_divider___024root__trace_full_0_sub_0\n"); );
    Vnewton_raphson_divider__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Init
    uint32_t* const oldp VL_ATTR_UNUSED = bufp->oldp(vlSymsp->__Vm_baseCode);
    // Body
    bufp->fullIData(oldp+1,(vlSelfRef.newton_raphson_divider__DOT__i_divider),32);
    bufp->fullIData(oldp+2,(vlSelfRef.newton_raphson_divider__DOT__i_dividend),32);
    bufp->fullQData(oldp+3,(vlSelfRef.newton_raphson_divider__DOT__i_quotient),63);
    bufp->fullCData(oldp+5,(vlSelfRef.newton_raphson_divider__DOT__shift_count),5);
    bufp->fullCData(oldp+6,(vlSelfRef.newton_raphson_divider__DOT__iter_count),5);
    bufp->fullCData(oldp+7,(vlSelfRef.newton_raphson_divider__DOT__shift_dir),2);
    bufp->fullBit(oldp+8,(vlSelfRef.newton_raphson_divider__DOT__fault));
    bufp->fullCData(oldp+9,(vlSelfRef.newton_raphson_divider__DOT__state),4);
    bufp->fullBit(oldp+10,(vlSelfRef.clk));
    bufp->fullBit(oldp+11,(vlSelfRef.nRST));
    bufp->fullBit(oldp+12,(vlSelfRef.request));
    bufp->fullIData(oldp+13,(vlSelfRef.divider),32);
    bufp->fullIData(oldp+14,(vlSelfRef.dividend),32);
    bufp->fullIData(oldp+15,(vlSelfRef.quotient),32);
    bufp->fullBit(oldp+16,(vlSelfRef.ready));
}
