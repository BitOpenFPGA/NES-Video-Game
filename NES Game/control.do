vlib work 
vlog test.v
vlog leftA.v
vlog leftB.v
vsim colours -L altera_mf_ver leftA -L altera_mf_ver leftB
log {/*}
add wave {/*}

force KEY 0000
force CLOCK_50 1
run 10 ns 
force CLOCK_50 0
run 10 ns

force KEY 0001
force CLOCK_50 1
run 10 ns
force CLOCK_50 0 
run 10 ns

force enable 0000
force {CLOCK_50} 0 0ns, 1 {5ns} -r 10ns
run 33425ns

force enable 0001
force {CLOCK_50} 0 0ns, 1 {5ns} -r 10ns
run 33425ns
