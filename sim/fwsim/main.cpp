// 命令行运行器。每个场景在独立进程中运行，因为程序的全局变量
//（和真实开发板一样）每次程序启动只初始化一次。
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
    } else if (a == "--sketch" && i + 1 < argc) {  // 例如 --sketch remote=remote_v2
      const std::string spec = argv[++i];
      const auto eq = spec.find('=');
      if (eq == std::string::npos) {
        std::cerr << "--sketch 的格式应为 原名=替换名\n";
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
      std::cerr << "用法：fwsim --list | --sketches | [--out 目录] [--quiet] [--seed N] [--sketch 原名=替换名] 场景\n";
      return 2;
    }
  }
  if (name.empty()) {
    std::cerr << "用法：fwsim --list | --sketches | [--out 目录] [--quiet] [--seed N] [--sketch 原名=替换名] 场景\n";
    return 2;
  }
  const fwsim::ScenarioEntry* entry = nullptr;
  for (const auto& s : fwsim::scenarios())
    if (s.name == name) entry = &s;
  if (!entry) {
    std::cerr << "未知的场景 '" << name << "'（见 --list）\n";
    return 2;
  }

  fwsim::Simulation sim;
  sim.verbose = !quiet;
  if (seed >= 0) sim.seed(static_cast<uint32_t>(seed));
  if (!quiet) std::cout << "=== " << entry->name << ": " << entry->description << "\n";
  try {
    entry->run(sim);
  } catch (const std::exception& e) {
    std::cerr << entry->name << "：配置错误：" << e.what() << "\n";
    return 2;
  }
  int failed = 0;
  for (const auto& c : sim.checks()) failed += c.ok ? 0 : 1;
  const std::string reportName = seed >= 0 ? entry->name + "-seed" + std::to_string(seed) : entry->name;
  sim.writeReport(out, reportName, entry->description);
  if (quiet)
    for (const auto& c : sim.checks())
      if (!c.ok) std::cout << "  失败  " << c.text << "  (" << c.where << ")\n";
  std::cout << (sim.passed() ? "通过" : "失败") << "：" << entry->name << "（检查 "
            << sim.checks().size() - static_cast<size_t>(failed) << "/" << sim.checks().size()
            << " 项通过，模拟 " << sim.nowMs() / 1000.0 << " s）-> " << out << "/"
            << reportName << ".html\n";
  return sim.passed() ? 0 : 1;
}
