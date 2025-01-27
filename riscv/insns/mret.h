require_privilege(PRV_M);
set_pc_and_serialize(p->get_state()->mepc->read());
reg_t s = STATE.mstatus->read();
#ifdef FORCE_RISCV_ENABLE
printf("<<<<<<<<<<<update_exception_event mret>>>>>>>>>>>>>>>\n");
SimException exit_m(0x4e, 0, "exit_mret", p->get_state()->mepc->read());
update_exception_event(&exit_m);
#endif
reg_t prev_prv = get_field(s, MSTATUS_MPP);
reg_t prev_virt = get_field(s, MSTATUS_MPV);
if (prev_prv != PRV_M)
  s = set_field(s, MSTATUS_MPRV, 0);
s = set_field(s, MSTATUS_MIE, get_field(s, MSTATUS_MPIE));
s = set_field(s, MSTATUS_MPIE, 1);
s = set_field(s, MSTATUS_MPP, p->extension_enabled('U') ? PRV_U : PRV_M);
s = set_field(s, MSTATUS_MPV, 0);
if (ZICFILP_xLPE(prev_virt, prev_prv)) {
  STATE.elp = static_cast<elp_t>(get_field(s, MSTATUS_MPELP));
  s = set_field(s, MSTATUS_MPELP, elp_t::NO_LP_EXPECTED);
}
#ifndef FORCE_RISCV_ENABLE
STATE.mstatus->write(s);
if (STATE.mstatush) STATE.mstatush->write(s >> 32); // log mstatush change
#else
p->put_csr(CSR_MSTATUS, s);
if (STATE.mstatush) p->put_csr(CSR_MSTATUSH, s >> 32); // log mstatush change
#endif
p->set_privilege(prev_prv, prev_virt);
