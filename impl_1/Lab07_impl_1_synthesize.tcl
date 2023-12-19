if {[catch {

# define run engine funtion
source [file join {C:/lscc/radiant/2023.1} scripts tcl flow run_engine.tcl]
# define global variables
global para
set para(gui_mode) 1
set para(prj_dir) "Y:/Documents/Polarbear/College/Classes/Engineering/MicroPs/Lab07"
# synthesize IPs
# synthesize VMs
# synthesize top design
file delete -force -- Lab07_impl_1.vm Lab07_impl_1.ldc
run_engine_newmsg synthesis -f "Lab07_impl_1_lattice.synproj"
run_postsyn [list -a iCE40UP -p iCE40UP5K -t SG48 -sp High-Performance_1.2V -oc Industrial -top -w -o Lab07_impl_1_syn.udb Lab07_impl_1.vm] "Y:/Documents/Polarbear/College/Classes/Engineering/MicroPs/Lab07/impl_1/Lab07_impl_1.ldc"

} out]} {
   runtime_log $out
   exit 1
}
