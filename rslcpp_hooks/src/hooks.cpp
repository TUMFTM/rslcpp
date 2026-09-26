// Copyright 2026 Simon Sagmeister
#include "rslcpp_hooks/hooks.hpp"

#include <algorithm>
#include <utility>
namespace rslcpp::hooks
{
namespace
{
template <typename Hook>
struct HookList
{
  std::vector<std::pair<HookId, Hook>> entries;

  HookId add(HookId id, Hook && hook)
  {
    entries.emplace_back(id, std::move(hook));
    return id;
  }
  bool remove(HookId id)
  {
    auto it = std::find_if(
      entries.begin(), entries.end(), [id](const auto & entry) { return entry.first == id; });
    if (it == entries.end()) {
      return false;
    }
    entries.erase(it);
    return true;
  }
};

struct Registry
{
  HookList<JobStartedHook> job_started;
  HookList<StepHook> step_begin;
  HookList<StepHook> step_end;
  HookList<CallbackHook> callback_start;
  HookList<CallbackHook> callback_end;
  HookList<JobFinishedHook> job_finished;
  HookId next_id = 1;
};

/// One registry for the process. A Meyers singleton, like the time-delay backend's.
Registry & registry()
{
  static Registry instance;
  return instance;
}
}  // namespace

void remove_hook(HookId id)
{
  auto & r = registry();
  // An id lives in exactly one list; try them in turn.
  r.job_started.remove(id) || r.step_begin.remove(id) || r.step_end.remove(id) ||
    r.callback_start.remove(id) || r.callback_end.remove(id) || r.job_finished.remove(id);
}

Registration on_job_started(JobStartedHook hook)
{
  auto & r = registry();
  return Registration(r.job_started.add(r.next_id++, std::move(hook)));
}
Registration on_step_begin(StepHook hook)
{
  auto & r = registry();
  return Registration(r.step_begin.add(r.next_id++, std::move(hook)));
}
Registration on_step_end(StepHook hook)
{
  auto & r = registry();
  return Registration(r.step_end.add(r.next_id++, std::move(hook)));
}
Registration on_callback_start(CallbackHook hook)
{
  auto & r = registry();
  return Registration(r.callback_start.add(r.next_id++, std::move(hook)));
}
Registration on_callback_end(CallbackHook hook)
{
  auto & r = registry();
  return Registration(r.callback_end.add(r.next_id++, std::move(hook)));
}
Registration on_job_finished(JobFinishedHook hook)
{
  auto & r = registry();
  return Registration(r.job_finished.add(r.next_id++, std::move(hook)));
}

void job_started(const NodeList & nodes)
{
  for (auto & [id, hook] : registry().job_started.entries) {
    hook(nodes);
  }
}
void step_begin(std::int64_t sim_time_ns)
{
  for (auto & [id, hook] : registry().step_begin.entries) {
    hook(sim_time_ns);
  }
}
void step_end(std::int64_t sim_time_ns)
{
  for (auto & [id, hook] : registry().step_end.entries) {
    hook(sim_time_ns);
  }
}
void callback_start(const CallbackInfo & info)
{
  for (auto & [id, hook] : registry().callback_start.entries) {
    hook(info);
  }
}
void callback_end(const CallbackInfo & info)
{
  // Reverse order, so the interval of a hook registered first encloses all later ones.
  auto & entries = registry().callback_end.entries;
  for (auto it = entries.rbegin(); it != entries.rend(); ++it) {
    it->second(info);
  }
}
void job_finished()
{
  // Copy, since a hook may unregister itself (or others) while finishing.
  auto entries = registry().job_finished.entries;
  for (auto & [id, hook] : entries) {
    hook();
  }
}
}  // namespace rslcpp::hooks
