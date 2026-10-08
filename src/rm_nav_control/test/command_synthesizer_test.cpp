#include "rm_nav_control/command_synthesizer.hpp"

#include <cassert>

int main()
{
  rm_nav_control::CommandSynthesizer synthesizer;
  auto command = synthesizer.synthesize({4.0, -4.0, 4.0}, true);
  assert(command.vx == 3.0 && command.vy == -3.0 && command.wz == 3.14);
  command = synthesizer.synthesize({1.0, 1.0, 1.0}, false);
  assert(command.vx == 0.0 && command.vy == 0.0 && command.wz == 0.0);
  return 0;
}
