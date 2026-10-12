#pragma once

#include <cstdint>
#include <mutex>
#include <optional>
#include <utility>

namespace fsm {

class GoalLifecycleGuard {
 public:
  struct Token {
    uint64_t epoch;
    uint64_t goal_id;
  };

  // All callbacks below run under the non-recursive lifecycle mutex. They must
  // be short and bounded (no planning or blocking I/O) and must not call this
  // guard again. If a callback takes another lock, every caller must acquire
  // locks in lifecycle-then-other order; never enter this guard while holding
  // a lock that a callback may acquire.

  template <typename Callback>
  Token activate(uint64_t goal_id, Callback&& callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++epoch_;
    active_goal_id_ = goal_id;
    active_ = true;
    std::forward<Callback>(callback)();
    return Token{epoch_, active_goal_id_};
  }

  Token activate(uint64_t goal_id) {
    return activate(goal_id, [] {});
  }

  template <typename Callback>
  bool cancel(uint64_t requested_goal_id, Callback&& callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (requested_goal_id != 0 &&
        (!active_ || requested_goal_id != active_goal_id_)) {
      return false;
    }

    ++epoch_;
    active_goal_id_ = 0;
    active_ = false;
    std::forward<Callback>(callback)();
    return true;
  }

  bool cancel(uint64_t requested_goal_id) {
    return cancel(requested_goal_id, [] {});
  }

  template <typename Callback>
  bool commitIfCurrent(const Token& token, Callback&& callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!isCurrentLocked(token)) {
      return false;
    }
    std::forward<Callback>(callback)();
    return true;
  }

  template <typename Callback>
  bool runIfCurrent(const Token& token, Callback&& callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!isCurrentLocked(token)) {
      return false;
    }
    std::forward<Callback>(callback)();
    return true;
  }

  // Read epoch and goal ID atomically. An inactive guard has no current token.
  std::optional<Token> snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_) {
      return std::nullopt;
    }
    return Token{epoch_, active_goal_id_};
  }

  uint64_t activeGoalId() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return active_goal_id_;
  }

  bool isActive() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return active_;
  }

 private:
  bool isCurrentLocked(const Token& token) const {
    return active_ && token.epoch == epoch_ &&
           token.goal_id == active_goal_id_;
  }

  mutable std::mutex mutex_;
  uint64_t epoch_{0};
  uint64_t active_goal_id_{0};
  bool active_{false};
};

}  // namespace fsm
