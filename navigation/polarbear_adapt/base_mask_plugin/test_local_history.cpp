#include "local_history.hpp"
#include <stdexcept>
#include <iostream>
using arx_base_mask::LocalHistory;
static void check(bool value,const char* why){if(!value)throw std::runtime_error(why);}
int main(){
  LocalHistory h(5.,.65); const LocalHistory::Key a{10,20},b{11,20};
  h.hit(a,10.,10.); h.advance(14.99);
  check(h.at(a).state==LocalHistory::State::Occupied,"early expiry");
  h.hit(a,10.,14.99); h.advance(15.);
  check(h.at(a).state==LocalHistory::State::Free,"cache refreshed or expiry not free");
  h.hit(a,10.,15.1);
  check(h.at(a).state==LocalHistory::State::Free,"stale cache revived expired mark");
  h.through(a,10.1,15.2);
  check(h.at(a).state==LocalHistory::State::Free,"stale ray changed expired free cell");
  h.hit(a,15.3,15.3); h.advance(20.);
  check(h.at(a).state==LocalHistory::State::Occupied,"real fresh hit failed to refresh");
  h.through(a,20.,20.);
  check(h.at(a).state==LocalHistory::State::Free,"real through did not clear early");
  h.hit(a,15.3,20.);
  check(h.at(a).state==LocalHistory::State::Free,"cached hit resurrected cleared cell");
  h.hit(a,20.1,20.1);
  check(h.at(a).state==LocalHistory::State::Occupied,"new actual obstacle not restored");
  h.adopt(b,20.1); h.adopt(b,24.9); h.advance(25.1);
  check(h.at(b).state==LocalHistory::State::Free,"legacy column refresh on each cycle");
  h.hit(b,26.,25.1);
  check(h.at(b).state==LocalHistory::State::Free,"future timestamp accepted");
  h.advance(1.);
  check(h.at(a).state==LocalHistory::State::Unknown,"rollback revived/free history");
  h.hit(a,1.,1.);
  check(h.at(a).state==LocalHistory::State::Occupied,"fresh postrollback hit not accepted");
  h.advance(7.);check(h.at(b).state==LocalHistory::State::Free,"rollback hold did not expire");
  h.through({100,100},7.,7.);check(!h.contains({100,100}),"unseen ray manufactured obstacle history");
  h.erase(b); check(!h.contains(b),"rolling/chassis erase failed");
  std::cout<<"PASS history: expiry, cache, actual hit, through, legacy and rollback\n";
}
