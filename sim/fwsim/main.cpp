// Command-line runner. Each scenario runs in its own process because sketch
// globals (like on a real board) are initialised only once per program start.
#include "Simulation.h"
#include <cstdlib>
#include <cstring>
#include <iostream>

int main(int argc, char** argv) {
  std::string out = "build/fwsim", name;
  bool quiet = false;
  long seed = -1;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--list") {
      for (const auto& s : fwsim::scenarios()) std::cout << s.name << "\t" << s.description << "\n";
      return 0;
    } else if (a == "--sketches") {
      for (const auto& s : fwsim::sketchNames()) std::cout << s << "\n";
      return 0;
    } else if (a == "--quiet") {
      quiet = true;
    } else if (a == "--sketch" && i + 1 < argc) {  // e.g. --sketch remote=remote_v2
      const std::string spec = argv[++i];
      const auto eq = spec.find('=');
      if (eq == std::string::npos) {
        std::cerr << "--sketch expects ORIGINAL=REPLACEMENT\n";
        return 2;
      }
      fwsim::substituteSketch(spec.substr(0, eq), spec.substr(eq + 1));
    } else if (a == "--seed" && i + 1 < argc) {
      seed = std::strtol(argv[++i], nullptr, 10);
    } else if (a == "--out" && i + 1 < argc) {
      out = argv[++i];
    } else if (!a.empty() && a[0] != '-' && name.empty()) {
      name = a;
    } else {
      std::cerr << "usage: fwsim --list | --sketches | [--out DIR] [--quiet] [--seed N] [--sketch A=B] SCENARIO\n";
      return 2;
    }
  }
  if (name.empty()) {
    std::cerr << "usage: fwsim --list | --sketches | [--out DIR] [--quiet] [--seed N] [--sketch A=B] SCENARIO\n";
    return 2;
  }
  const fwsim::ScenarioEntry* entry = nullptr;
  for (const auto& s : fwsim::scenarios())
    if (s.name == name) entry = &s;
  if (!entry) {
    std::cerr << "unknown scenario '" << name << "' (see --list)\n";
    return 2;
  }

  fwsim::Simulation sim;
  sim.verbose = !quiet;
  if (seed >= 0) sim.seed(static_cast<uint32_t>(seed));
  if (!quiet) std::cout << "=== " << entry->name << ": " << entry->description << "\n";
  try {
    entry->run(sim);
  } catch (const std::exception& e) {
    std::cerr << entry->name << ": setup error: " << e.what() << "\n";
    return 2;
  }
  int failed = 0;
  for (const auto& c : sim.checks()) failed += c.ok ? 0 : 1;
  const std::string reportName = seed >= 0 ? entry->name + "-seed" + std::to_string(seed) : entry->name;
  sim.writeReport(out, reportName, entry->description);
  if (quiet)
    for (const auto& c : sim.checks())
      if (!c.ok) std::cout << "  FAIL  " << c.text << "  (" << c.where << ")\n";
  std::cout << (sim.passed() ? "PASS" : "FAIL") << ": " << entry->name << " ("
            << sim.checks().size() - static_cast<size_t>(failed) << "/" << sim.checks().size()
            << " checks, " << sim.nowMs() / 1000.0 << " s simulated) -> " << out << "/"
            << reportName << ".html\n";
  return sim.passed() ? 0 : 1;
}
