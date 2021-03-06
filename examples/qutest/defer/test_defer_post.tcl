# test-script for QUTest unit testing harness
# see https://www.state-machine.com/qtools/qutest.html

# preamble...
proc on_reset {} {
    expect_pause
    glb_filter SM AO QF EQ
    continue
    expect "===RTC===> St-Init  Obj=l_tserver,State=QHsm::top->TServer::idle"
    expect "%timestamp AO-RCllA Obj=l_tserver,Que=l_tserver.m_requestQueue"
    expect "===RTC===> St-Entry Obj=l_tserver,State=TServer::idle"
    expect "%timestamp Init===> Obj=l_tserver,State=TServer::idle"
    current_obj SM_AO l_tserver
}

# tests...
test "NEW_REQUEST_SIG->l_tserver"
post NEW_REQUEST_SIG
expect "%timestamp QF-New   Sig=NEW_REQUEST_SIG,*"
#expect "%timestamp MP-Get   Obj=smlPoolSto,*"
expect "%timestamp AO-Post  Sdr=QS_RX,Obj=l_tserver,Evt<Sig=NEW_REQUEST_SIG,Pool=1,Ref=1>,*"
expect "%timestamp AO-GetL  Obj=l_tserver,Evt<Sig=NEW_REQUEST_SIG,Pool=1,Ref=1>"
expect "%timestamp Disp===> Obj=l_tserver,Sig=NEW_REQUEST_SIG,State=TServer::idle"
expect "%timestamp QF-NewRf Evt<Sig=NEW_REQUEST_SIG,Pool=1,Ref=2>"
#expect "%timestamp TE0-Arm  Obj=l_tserver.m_receivedEvt,AO=l_tserver,Tim=100,Int=0"
expect "===RTC===> St-Entry Obj=l_tserver,State=TServer::receiving"
expect "%timestamp ===>Tran Obj=l_tserver,Sig=NEW_REQUEST_SIG,State=TServer::idle->TServer::receiving"
expect "%timestamp QF-gcA   Evt<Sig=NEW_REQUEST_SIG,Pool=1,Ref=2>"
expect "%timestamp Trg-Done QS_RX_EVENT"

test "NEW_REQUEST_SIG->l_tserver (defer)" -noreset
post NEW_REQUEST_SIG
expect "%timestamp QF-New   Sig=NEW_REQUEST_SIG,*"
#expect "%timestamp MP-Get   Obj=smlPoolSto,*"
expect "%timestamp AO-Post  Sdr=QS_RX,Obj=l_tserver,Evt<Sig=NEW_REQUEST_SIG,Pool=1,Ref=1>,*"
expect "%timestamp AO-GetL  Obj=l_tserver,Evt<Sig=NEW_REQUEST_SIG,Pool=1,Ref=1>"
expect "%timestamp Disp===> Obj=l_tserver,Sig=NEW_REQUEST_SIG,State=TServer::receiving"
expect "%timestamp EQ-Post  Obj=l_tserver.m_requestQueue,Evt<Sig=NEW_REQUEST_SIG,Pool=1,Ref=2>,*"
expect "%timestamp AO-Defer Obj=l_tserver,Que=l_tserver.m_requestQueue,Evt<Sig=NEW_REQUEST_SIG,Pool=1,Ref=2>"
expect "%timestamp =>Intern Obj=l_tserver,Sig=NEW_REQUEST_SIG,State=TServer::busy"
expect "%timestamp QF-gcA   Evt<Sig=NEW_REQUEST_SIG,Pool=1,Ref=2>"
expect "%timestamp Trg-Done QS_RX_EVENT"

test "RECEIVED_SIG/AUTHORIZED_SIG->l_tserver (recall)" -noreset
post RECEIVED_SIG
expect "%timestamp QF-New   Sig=RECEIVED_SIG,*"
#expect "%timestamp MP-Get   Obj=smlPoolSto,*"
expect "%timestamp AO-Post  Sdr=QS_RX,Obj=l_tserver,Evt<Sig=RECEIVED_SIG,Pool=1,Ref=1>,*"
expect "%timestamp AO-GetL  Obj=l_tserver,Evt<Sig=RECEIVED_SIG,Pool=1,Ref=1>"
expect "%timestamp Disp===> Obj=l_tserver,Sig=RECEIVED_SIG,State=TServer::receiving"
#expect "%timestamp TE0-Dis  Obj=l_tserver.m_receivedEvt,AO=l_tserver,Tim=100,Int=0"
expect "===RTC===> St-Exit  Obj=l_tserver,State=TServer::receiving"
#expect "%timestamp TE0-Arm  Obj=l_tserver.m_authorizedEvt,AO=l_tserver,Tim=200,Int=0"
expect "===RTC===> St-Entry Obj=l_tserver,State=TServer::authorizing"
expect "%timestamp ===>Tran Obj=l_tserver,Sig=RECEIVED_SIG,State=TServer::receiving->TServer::authorizing"
expect "%timestamp QF-gc    Evt<Sig=RECEIVED_SIG,Pool=1,Ref=1>"
#expect "%timestamp MP-Put   Obj=smlPoolSto,*"
expect "%timestamp Trg-Done QS_RX_EVENT"
post AUTHORIZED_SIG
expect "%timestamp QF-New   Sig=AUTHORIZED_SIG,*"
#expect "%timestamp MP-Get   Obj=smlPoolSto,*"
expect "%timestamp AO-Post  Sdr=QS_RX,Obj=l_tserver,Evt<Sig=AUTHORIZED_SIG,Pool=1,Ref=1>,*"
expect "%timestamp AO-GetL  Obj=l_tserver,Evt<Sig=AUTHORIZED_SIG,Pool=1,Ref=1>"
expect "%timestamp Disp===> Obj=l_tserver,Sig=AUTHORIZED_SIG,State=TServer::authorizing"
#expect "%timestamp TE0-Dis  Obj=l_tserver.m_authorizedEvt,AO=l_tserver,Tim=200,Int=0"
expect "===RTC===> St-Exit  Obj=l_tserver,State=TServer::authorizing"
expect "%timestamp QF-DelRf Evt<Sig=NEW_REQUEST_SIG,Pool=1,Ref=1>"
expect "%timestamp QF-gc    Evt<Sig=NEW_REQUEST_SIG,Pool=1,Ref=1>"
#expect "%timestamp MP-Put   Obj=smlPoolSto,*"
expect "===RTC===> St-Exit  Obj=l_tserver,State=TServer::busy"
expect "%timestamp EQ-GetL  Obj=l_tserver.m_requestQueue,Evt<Sig=NEW_REQUEST_SIG,Pool=1,Ref=1>"
expect "%timestamp AO-LIFO  Obj=l_tserver,Evt<Sig=NEW_REQUEST_SIG,Pool=1,Ref=2>,*"
expect "%timestamp AO-RCall Obj=l_tserver,Que=l_tserver.m_requestQueue,Evt<Sig=NEW_REQUEST_SIG,Pool=1,Ref=1>"
expect "===RTC===> St-Entry Obj=l_tserver,State=TServer::idle"
expect "%timestamp ===>Tran Obj=l_tserver,Sig=AUTHORIZED_SIG,State=TServer::authorizing->TServer::idle"
expect "%timestamp QF-gc    Evt<Sig=AUTHORIZED_SIG,Pool=1,Ref=1>"
#expect "%timestamp MP-Put   Obj=smlPoolSto,*"
expect "%timestamp AO-GetL  Obj=l_tserver,Evt<Sig=NEW_REQUEST_SIG,Pool=1,Ref=1>"
expect "%timestamp Disp===> Obj=l_tserver,Sig=NEW_REQUEST_SIG,State=TServer::idle"
expect "%timestamp QF-NewRf Evt<Sig=NEW_REQUEST_SIG,Pool=1,Ref=2>"
#expect "%timestamp TE0-Arm  Obj=l_tserver.m_receivedEvt,AO=l_tserver,Tim=100,Int=0"
expect "===RTC===> St-Entry Obj=l_tserver,State=TServer::receiving"
expect "%timestamp ===>Tran Obj=l_tserver,Sig=NEW_REQUEST_SIG,State=TServer::idle->TServer::receiving"
expect "%timestamp QF-gcA   Evt<Sig=NEW_REQUEST_SIG,Pool=1,Ref=2>"
expect "%timestamp Trg-Done QS_RX_EVENT"


# the end
end
