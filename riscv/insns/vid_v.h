// vmpopc rd, vs2, vm
require(P.VU.vsew >= e8 && P.VU.vsew <= e64);
require_vector(true);
reg_t sew = P.VU.vsew;
reg_t rd_num = insn.rd();
require_align(rd_num, P.VU.vflmul);
require_vm;

for (reg_t i = P.VU.vstart->read() ; i < P.VU.vl->read(); ++i) {
  VI_LOOP_ELEMENT_SKIP();

#ifndef FORCE_RISCV_ENABLE
  switch (sew) {
  case e8:
    P.VU.elt<uint8_t>(rd_num, i, true) = i;
    break;
  case e16:
    P.VU.elt<uint16_t>(rd_num, i, true) = i;
    break;
  case e32:
    P.VU.elt<uint32_t>(rd_num, i, true) = i;
    break;
  default:
    P.VU.elt<uint64_t>(rd_num, i, true) = i;
    break;
  }
#else
  switch (sew) {
  case e8:
    P.VU.elt_do_callback<uint8_t>(rd_num, i, true) = i;
    break;
  case e16:
    P.VU.elt_do_callback<uint16_t>(rd_num, i, true) = i;
    break;
  case e32:
    P.VU.elt_do_callback<uint32_t>(rd_num, i, true) = i;
    break;
  default:
    P.VU.elt_do_callback<uint64_t>(rd_num, i, true) = i;
    break;
  }
#endif
}

P.VU.vstart->write(0);
