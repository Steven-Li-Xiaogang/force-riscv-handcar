require_extension('S');
reg_t prev_hstatus = STATE.hstatus->read();
if (STATE.v) {
  if (STATE.prv == PRV_U || get_field(prev_hstatus, HSTATUS_VTSR))
    require_novirt();
} else {
  require_privilege(get_field(STATE.mstatus->read(), MSTATUS_TSR) ? PRV_M : PRV_S);
}
reg_t next_pc = p->get_state()->sepc->read();
set_pc_and_serialize(next_pc);
reg_t s = STATE.sstatus->read();
#ifdef FORCE_RISCV_ENABLE
printf("<<<<<<<<<<<update_exception_event sret>>>>>>>>>>>>>>>\n");
SimException exit_s(0x4e, 0, "exit_sret", next_pc);
update_exception_event(&exit_s);
#endif
reg_t prev_prv = get_field(s, MSTATUS_SPP);
s = set_field(s, MSTATUS_SIE, get_field(s, MSTATUS_SPIE));
s = set_field(s, MSTATUS_SPIE, 1);
s = set_field(s, MSTATUS_SPP, PRV_U);
bool prev_virt = STATE.v;
if (!STATE.v) {
  if (p->extension_enabled('H')) {
    prev_virt = get_field(prev_hstatus, HSTATUS_SPV);
    reg_t new_hstatus = set_field(prev_hstatus, HSTATUS_SPV, 0);
    STATE.hstatus->write(new_hstatus);
  }

  STATE.mstatus->write(set_field(STATE.mstatus->read(), MSTATUS_MPRV, 0));
}
if (ZICFILP_xLPE(prev_virt, prev_prv)) {
  STATE.elp = static_cast<elp_t>(get_field(s, SSTATUS_SPELP));
  s = set_field(s, SSTATUS_SPELP, elp_t::NO_LP_EXPECTED);
}
#ifndef FORCE_RISCV_ENABLE
STATE.sstatus->write(s);
#else
p->put_csr(CSR_SSTATUS, s);
#endif
p->set_privilege(prev_prv, prev_virt);
