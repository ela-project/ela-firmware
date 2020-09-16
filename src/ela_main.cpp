#include "analyzer.h"

int main() {
  analyzer_agent.init();

  while (1) {
    analyzer_agent.idle();
  }

  return 0;
}
