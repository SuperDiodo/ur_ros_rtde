#ifndef TRAJECTORY_TIME_PLANNING_H
#define TRAJECTORY_TIME_PLANNING_H

#include <vector>
#include <stdint.h>
#include <cmath>
#include <array>
#include <algorithm>
#include <iostream>
#include <memory>

static constexpr size_t NUM_JOINTS = 6;
typedef std::array<double, NUM_JOINTS> Array6d;
typedef std::array<size_t, NUM_JOINTS> Array6size;
static const Array6d joint_speed_limits = {2.09, 2.09, 2.09, M_PI, M_PI, M_PI};
static const Array6d ZEROS = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
static const Array6size ZERO_SIZES = {0, 0, 0, 0, 0, 0};
static const double EPSILON = 0.0000001;

template <typename T> T SQR(const T & t) {return t * t; }
template <typename T> T SQRT(const T & t) {return std::sqrt(t); }

inline std::ostream & operator<<(std::ostream & o, const Array6d & a)
{
  o << "{";
  for (double v : a)
    o << v << " ";
  o << "}";
  return o;
}

inline Array6d operator/(const Array6d & a, const double o)
{
  Array6d result;
  for (size_t i = 0; i < 6; i++)
    result[i] = a[i] / o;
  return result;
}

inline Array6d operator-(const Array6d & a, const Array6d & b)
{
  Array6d result;
  for (size_t i = 0; i < NUM_JOINTS; i++)
    result[i] = a[i] - b[i];
  return result;
}

template <typename T> T sign(const T & t)
{
  if (t < 0) return -1;
  if (t > 0) return 1;
  return 0;
}

inline std::vector<double> Array6dToVector(const Array6d & a)
{
  return std::vector<double>(a.begin(), a.end());
}

// returns at most two elements
inline std::vector<double> solve_quadratic(const double a, const double b, const double c)
{
  const double delta = SQR(b) - 4.0 * a * c;
  if (delta < 0.0)
    return std::vector<double>();

  std::vector<double> result;
  const double x1 = (-b + std::sqrt(delta)) / (2.0 * a);
  result.push_back(x1);
  const double x2 = (-b - std::sqrt(delta)) / (2.0 * a);
  result.push_back(x2);

  return result;
}

struct JointTrajFrag
{
  double start;
  double end;
  double v_start;
  double v_end;
  double accel;
  double time_start;
  double time_end;

  enum class Primitive
  {
    UNDEFINED,
    ACCEL_PLATEAU,
    PLATEAU_ACCEL,
    TRAPEZOID,
    RAMP_UD,
    RAMP_UUD,
    RAMP_UDD,
    END,
  };

  struct Comment
  {
    uint64_t waypoint = 0;
    uint64_t joint = 0;
    Primitive primitive = Primitive::UNDEFINED;
    std::string action;

    Comment() {}

    Comment(const uint64_t waypoint, const Primitive primitive, const std::string & action):
      waypoint(waypoint), primitive(primitive), action(action) {}

    static std::string primitive_to_string(const Primitive primitive)
    {
      switch (primitive)
      {
        case Primitive::UNDEFINED:     return "undefined";
        case Primitive::ACCEL_PLATEAU: return "acc-pla";
        case Primitive::PLATEAU_ACCEL: return "pla-acc";
        case Primitive::TRAPEZOID:     return "trapezoid";
        case Primitive::RAMP_UD:       return "ramp-ud";
        case Primitive::RAMP_UUD:      return "ramp-uud";
        case Primitive::RAMP_UDD:      return "ramp-udd";
        case Primitive::END:           return "end";
      }

      std::cerr << "JointTrajFrag::Comment::primitive_to_string: ???" << std::endl;
      return "???";
    }

    std::string to_string() const
    {
      return std::to_string(waypoint) + "_j" + std::to_string(joint) + "_" +
             primitive_to_string(primitive) + "_" + action;
    }

    std::shared_ptr<const Comment> withWaypoint(const uint64_t waypoint) const
    {
      std::shared_ptr<Comment> result = std::make_shared<Comment>(*this);
      result->waypoint = waypoint;
      return result;
    }

    std::shared_ptr<const Comment> withJoint(const uint64_t joint) const
    {
      std::shared_ptr<Comment> result = std::make_shared<Comment>(*this);
      result->joint = joint;
      return result;
    }

    std::shared_ptr<const Comment> withAction(const std::string & action) const
    {
      std::shared_ptr<Comment> result = std::make_shared<Comment>(*this);
      result->action = action;
      return result;
    }

    std::shared_ptr<const Comment> withPrimitive(const Primitive primitive) const
    {
      std::shared_ptr<Comment> result = std::make_shared<Comment>(*this);
      result->primitive = primitive;
      return result;
    }
  };

  std::shared_ptr<const Comment> comment;

  JointTrajFrag(const double start, const double end, const double v_start, const double v_end,
                const double accel, const double time_start, const double time_end, const std::shared_ptr<const Comment> comment)
    :start(start), end(end), v_start(v_start), v_end(v_end), accel(accel), time_start(time_start), time_end(time_end),
     comment(comment) {}
};

struct JointTrajSample
{
  double s;
  double t;
  std::shared_ptr<const JointTrajFrag::Comment> comment;

  JointTrajSample() {}
  JointTrajSample(double s, double t, const std::shared_ptr<const JointTrajFrag::Comment> comment): s(s), t(t), comment(comment) {}
};

struct TrajectorySample
{
  Array6d p;
  Array6d v;
  double t;
  bool reached = false;
  std::array<std::shared_ptr<const JointTrajFrag::Comment>, NUM_JOINTS> comments;
};

inline
bool build_traj_primitive_accel_plateau(const double start, const double end, const double v_start,
                                        const double v_end, const double time_start, const double time_end,
                                        const double v_max, const double a_max, std::vector<JointTrajFrag> & fragments)
{
  //       /--------v1
  //      /a
  //  v_0/
  //     |t0|   t1   |
  //     |s0|   s1   |
  // unknowns: t0 t1 s0 s1 a

  (void)(v_max); // prevent unused warning
  fragments.clear();

  typedef JointTrajFrag::Primitive Primitive;
  typedef JointTrajFrag::Comment Comment;

  static const std::shared_ptr<const Comment> comment = Comment().withPrimitive(Primitive::ACCEL_PLATEAU);
  static const std::shared_ptr<const Comment> comment_accel = comment->withAction("a");
  static const std::shared_ptr<const Comment> comment_plateau = comment->withAction("p");
  static const std::shared_ptr<const Comment> comment_nothing = comment->withAction("n");

  if (std::abs(start - end) < EPSILON && std::abs(v_end - v_start) < EPSILON)
  {
    if (time_end > time_start)
      fragments.push_back(JointTrajFrag(start, end, v_start, v_end, 0.0, time_start, time_end, comment_nothing));
    return true;  // nothing to do
  }
  if (time_end - time_start < EPSILON)
    return false; // if we have something to do but no time fail

  const double s0a = (time_end - time_start) * v_end - (end - start);
  double t0 = (std::abs(v_end - v_start) < EPSILON) ? 0.0 :
              ((2.0 * s0a) / (v_end - v_start));
  if (t0 < -EPSILON)
    return false;
  if (t0 < 0.0) t0 = 0.0;
  const double a = (t0 < EPSILON) ? 0.0 :
                   ((v_end - v_start) / t0);
  double t1 = (time_end - time_start) - t0;
  if (t1 < -EPSILON)
    return false;
  if (t1 < 0.0) t1 = 0.0;
  const double s1 = t1 * v_end;
  const double s0 = v_start * t0 + 0.5 * a * SQR(t0);

  const double end_check = start + v_start * t0 + 0.5 * a * SQR(t0) + s1;

//  std::cout << "accel_plateau: start " << start << " end " << end << " v_start " << v_start << " v_end " << v_end <<
//               " time_start " << time_start << " time_end " << time_end << " v_max " << v_max << " a_max " << a_max << std::endl;
//  std::cout << "s0 " << s0 << " a " << a << " t0 " << t0 << " s1 " << s1 << " t1 " << t1 << std::endl;
//  std::cout << "s_check " << end_check << " == end " << end << std::endl;

  if (std::isnan(a) || std::isinf(a) || std::isnan(t0) || std::isinf(t0) || std::isnan(t1) || std::isinf(t1))
    return false;

  if (std::abs(a) > a_max + EPSILON)
    return false;
  if (std::abs(end_check - end) > 10.0 * EPSILON)
    return false; // self consistency fail

  if (t0 != 0.0)
    fragments.push_back(JointTrajFrag(start, start + s0, v_start, v_end, a, time_start, time_start + t0, comment_accel));
  if (t1 != 0.0)
    fragments.push_back(JointTrajFrag(start + s0, end, v_end, v_end, 0.0, time_start + t0, time_end, comment_plateau));

  // ensure that the last fragment ends exactly at end time
  if (!fragments.empty())
    fragments.back().time_end = time_end;

  return true;
}

inline
bool build_traj_primitive_plateau_accel(const double start, const double end, const double v_start,
                                        const double v_end, const double time_start, const double time_end,
                                        const double v_max, const double a_max, std::vector<JointTrajFrag> & fragments)
{
  //            /v1
  //           /a
  //  v0------/
  //  |t0     |t1|
  //  |s0     |s1|
  // unknowns: t0 t1 s0 s1 a

  (void)(v_max); // prevent unused warning

  fragments.clear();

  typedef JointTrajFrag::Primitive Primitive;
  typedef JointTrajFrag::Comment Comment;

  static const std::shared_ptr<const Comment> comment = Comment().withPrimitive(Primitive::PLATEAU_ACCEL);
  static const std::shared_ptr<const Comment> comment_accel = comment->withAction("a");
  static const std::shared_ptr<const Comment> comment_plateau = comment->withAction("p");
  static const std::shared_ptr<const Comment> comment_nothing = comment->withAction("n");

  if (std::abs(start - end) < EPSILON && std::abs(v_end - v_start) < EPSILON)
  {
    if (time_end > time_start)
      fragments.push_back(JointTrajFrag(start, end, v_start, v_end, 0.0, time_start, time_end, comment_nothing));
    return true;  // nothing to do
  }
  if (time_end - time_start < EPSILON)
    return false; // if we have something to do but no time fail

  const double s1a = (end - start) - (time_end - time_start) * v_start;
  double t1 = (std::abs(v_end - v_start) < EPSILON) ? 0.0 :
                    ((2.0 * s1a) / (v_end - v_start));
  if (t1 < -EPSILON)
    return false;
  if (t1 < 0.0) t1 = 0.0;
  double a = (t1 < EPSILON) ? 0.0 :
             ((v_end - v_start) / t1);
  double t0 = (time_end - time_start) - t1;
  if (t0 < -EPSILON)
    return false;
  if (t0 < 0.0) t0 = 0.0;
  const double s0 = t0 * v_start;

  const double end_check = start + s0 + 0.5 * a * SQR(t1) + v_start * t1;

//  std::cout << "start " << start << " end " << end << " v_start " << v_start << " v_end " << v_end <<
//               " time_start " << time_start << " time_end " << time_end << " v_max " << v_max << " a_max " << a_max << std::endl;
//  std::cout << "s0 " << s0 << " a " << a << " t0 " << t0 << " s1 " << s1 << " t1 " << t1 << std::endl;
//  std::cout << "s_check " << end_check << " == end " << end << std::endl;

  if (std::isnan(a) || std::isinf(a) || std::isnan(t0) || std::isinf(t0) || std::isnan(t1) || std::isinf(t1))
    return false;

  if (std::abs(a) > a_max + EPSILON)
    return false;
  if (a > a_max) a = a_max;
  if (a < -a_max) a = -a_max;

  if (std::abs(end_check - end) > 10.0 * EPSILON)
    return false; // self consistency fail

  if (t0 != 0.0)
    fragments.push_back(JointTrajFrag(start, start + s0, v_start, v_start, 0.0, time_start, time_start + t0, comment_plateau));
  if (t1 != 0.0)
    fragments.push_back(JointTrajFrag(start + s0, end, v_start, v_end, a, time_start + t0, time_end, comment_accel));

  // ensure that the last fragment ends exactly at end time
  if (!fragments.empty())
    fragments.back().time_end = time_end;

  return true;
}

inline
bool build_traj_primitive_trapezoid(const double start, const double end, const double v_start,
                                    const double v_end, const double time_start, const double time_end,
                                    const double v_max, const double a_max, std::vector<JointTrajFrag> & fragments)
{
  //    /---v_max---\-a
  //   /a            \-a
  //  /               \-a
  //  |t1|   t2     |t1|
  //  |s1|   s2     |s1|

  fragments.clear();

  typedef JointTrajFrag::Primitive Primitive;
  typedef JointTrajFrag::Comment Comment;

  static const std::shared_ptr<const Comment> comment = Comment().withPrimitive(Primitive::TRAPEZOID);

  static const std::shared_ptr<const Comment> comment_accel1 = comment->withAction("a1");
  static const std::shared_ptr<const Comment> comment_plateau2 = comment->withAction("p2");
  static const std::shared_ptr<const Comment> comment_accel3 = comment->withAction("a3");
  static const std::shared_ptr<const Comment> comment_nothing = comment->withAction("n");

  if (std::abs(v_end - v_start) >= EPSILON)
    return false; // only v_end == v_start is supported
  if (std::abs(start - end) < EPSILON)
  {
    if (time_end > time_start)
      fragments.push_back(JointTrajFrag(start, end, v_start, v_end, 0.0, time_start, time_end, comment_nothing));
    return true;  // nothing to do
  }
  if (time_end - time_start < EPSILON)
    return false; // if we have something to do but no time fail

  const double s = end - start;

  const double t = time_end - time_start;
  const double v2 = v_max * sign(s);

  double t1 = (std::abs(v2 - v_start) < EPSILON) ? 0.0 : ((t * v2 - s) / (v2 - v_start));
  if (t1 < -EPSILON)
    return false;
  if (t1 < 0.0) t1 = 0.0;
  const double a = (t1 < EPSILON) ? 0.0 : ((v2 - v_start) / t1);
  if (std::abs(a) > a_max)
    return false; // too high acceleration required

  double t2 = t - 2.0 * t1;
  if (t2 < -EPSILON)
    return false;
  if (t2 < 0.0) t2 = 0.0;

  const double s1 = 0.5 * a * SQR(t1) + v_start * t1;
  const double s2 = v2 * t2;
  const double s3 = 0.5 * a * SQR(t1) + v_start * t1;

  const double end_check = start + s1 + s2 + s3;

  if (std::abs(end_check - end) > 10.0 * EPSILON)
    return false; // self consistency fail

  if (t1 != 0.0)
    fragments.push_back(JointTrajFrag(start, start + s1, v_start, v2, a, time_start, time_start + t1, comment_accel1));
  if (t2 != 0.0)
    fragments.push_back(JointTrajFrag(start + s1, end - s1, v2, v2, 0.0, time_start + t1,
                                      time_start + t1 + t2, comment_plateau2));
  if (t1 != 0.0)
    fragments.push_back(JointTrajFrag(end - s1, end, v2, v_start, -a, time_start + t1 + t2,
                                      time_end, comment_accel3));

  // ensure that the last fragment ends exactly at end time
  if (!fragments.empty())
    fragments.back().time_end = time_end;

  return true;
}

inline
bool build_traj_primitive_ramp_up_down(const double start, const double end, const double v_start,
                                       const double v_end, const double time_start, const double time_end,
                                       const double v_max, const double a_max, std::vector<JointTrajFrag> & fragments)
{
  //      /v2\-a
  //     /a   \-a
  //  v0/      \v0
  //  |    t    |
  //  |    s    |

  fragments.clear();

  typedef JointTrajFrag::Primitive Primitive;
  typedef JointTrajFrag::Comment Comment;

  static const std::shared_ptr<const Comment> comment = Comment().withPrimitive(Primitive::RAMP_UD);

  static const std::shared_ptr<const Comment> comment_accel0 = comment->withAction("a0");
  static const std::shared_ptr<const Comment> comment_accel1 = comment->withAction("a1");
  static const std::shared_ptr<const Comment> comment_nothing = comment->withAction("n");

  if (std::abs(v_end - v_start) >= EPSILON)
    return false; // only v_end == v_start is supported
  if (std::abs(start - end) < EPSILON)
  {
    if (time_end > time_start)
      fragments.push_back(JointTrajFrag(start, end, v_start, v_end, 0.0, time_start, time_end, comment_nothing));
    return true;  // nothing to do
  }
  if (time_end - time_start < EPSILON)
    return false; // if we have something to do but no time fail

  const double s = end - start;
  const double t = time_end - time_start;
  const double v_mean = s / t;
  double v2 = 2.0 * v_mean - v_start;
  if (std::abs(v2) > v_max + EPSILON)
    return false;
  if (v2 > v_max) v2 = v_max;
  if (v2 < -v_max) v2 = -v_max;
  double a = (v2 - v_start) / (t / 2.0);
  if (std::abs(a) > a_max + EPSILON)
    return false;
  if (a > a_max) a = a_max;
  if (a < -a_max) a = -a_max;

  const double t0 = t / 2.0;
  const double s0 = 0.5 * a * SQR(t0) + v_start * t0;

  const double end_check = start + v_start * t + 2.0 * 0.5 * a * SQR(t / 2.0);

  if (std::abs(end_check - end) > 10.0 * EPSILON)
    return false; // self consistency fail

  fragments.push_back(JointTrajFrag(start, start + s0, v_start, v2, a, time_start, time_start + t0, comment_accel0));
  fragments.push_back(JointTrajFrag(start + s0, end, v2, v_start, -a, time_start + t0, time_end, comment_accel1));

  return true;
}

inline
bool build_traj_primitive_ramp_up_up_down(const double start, const double end, const double v_start,
                                          const double v_end, const double time_start, const double time_end,
                                          const double v_max, const double a_max, std::vector<JointTrajFrag> & fragments)
{
  //     a/\-a
  //     /  \v3
  //  v0/amax
  //  |t0| t1 |
  //  |s0| t1 |

  fragments.clear();

  typedef JointTrajFrag::Primitive Primitive;
  typedef JointTrajFrag::Comment Comment;

  static const std::shared_ptr<const Comment> comment = Comment().withPrimitive(Primitive::RAMP_UUD);

  static const std::shared_ptr<const Comment> comment_accel0 = comment->withAction("u");
  static const std::shared_ptr<const Comment> comment_nothing = comment->withAction("n");

  if (std::abs(start - end) < EPSILON)
  {
    if (time_end > time_start)
      fragments.push_back(JointTrajFrag(start, end, v_start, v_end, 0.0, time_start, time_end, comment_nothing));
    return true;  // nothing to do
  }
  if (time_end - time_start < EPSILON)
    return false; // if we have something to do but no time fail

  const double t = time_end - time_start;

  if (std::abs(v_end) < std::abs(v_start)) // only usable if speed increasing
    return false;

  const double signed_a_max = a_max * sign(v_end + v_start);
  double t0 = std::abs(v_end - v_start) / a_max;

  if (t0 > t + EPSILON)
    return false; // fail
  if (t0 > t)
    t0 = t;
  const double s0 = 0.5 * signed_a_max * SQR(t0) + v_start * t0;

  std::vector<JointTrajFrag> ramp_ud_fragments;
  if (!build_traj_primitive_ramp_up_down(start + s0, end, v_end, v_end, time_start + t0, time_end, v_max, a_max, ramp_ud_fragments))
    if (!build_traj_primitive_trapezoid(start + s0, end, v_end, v_end, time_start + t0, time_end, v_max, a_max, ramp_ud_fragments))
      return false; // fail

  const double s1 = ramp_ud_fragments.empty() ? 0.0 : (ramp_ud_fragments.back().end - ramp_ud_fragments.front().start);
  const double end_check = start + s0 + s1;

  if (std::abs(end_check - end) > 10.0 * EPSILON)
    return false; // self consistency fail

  if (t0 != 0.0)
    fragments.push_back(JointTrajFrag(start, start + s0, v_start, v_end, signed_a_max, time_start, time_start + t0, comment_accel0));
  fragments.insert(fragments.end(), ramp_ud_fragments.begin(), ramp_ud_fragments.end());

  return true;
}

inline
bool build_traj_primitive_ramp_up_down_down(const double start, const double end, const double v_start,
                                            const double v_end, const double time_start, const double time_end,
                                            const double v_max, const double a_max, std::vector<JointTrajFrag> & fragments)
{
  //     a/\-a
  //   v0/  \v3
  //         \-amax
  //   | t0 |t1|
  //   | s0 |t1|

  fragments.clear();

  typedef JointTrajFrag::Primitive Primitive;
  typedef JointTrajFrag::Comment Comment;

  static const std::shared_ptr<const Comment> comment = Comment().withPrimitive(Primitive::RAMP_UDD);

  static const std::shared_ptr<const Comment> comment_accel0 = comment->withAction("d");
  static const std::shared_ptr<const Comment> comment_nothing = comment->withAction("n");

  if (std::abs(start - end) < EPSILON)
  {
    if (time_end > time_start)
      fragments.push_back(JointTrajFrag(start, end, v_start, v_end, 0.0, time_start, time_end, comment_nothing));
    return true;  // nothing to do
  }
  if (time_end - time_start < EPSILON)
    return false; // if we have something to do but no time fail

  const double t = time_end - time_start;

  if (std::abs(v_end) > std::abs(v_start)) // only usable if speed decreasing
    return false;

  const double signed_a_max = a_max * sign(v_end + v_start);
  double t1 = std::abs(v_start - v_end) / a_max;
  if (t1 > t + EPSILON)
    return false; // fail
  if (t1 > t)
    t1 = t;
  const double s1 = 0.5 * signed_a_max * SQR(t1) + v_end * t1;

  std::vector<JointTrajFrag> ramp_ud_fragments;
  if (!build_traj_primitive_ramp_up_down(start, end - s1, v_start, v_start, time_start, time_end - t1, v_max, a_max, ramp_ud_fragments))
    if (!build_traj_primitive_trapezoid(start, end - s1, v_start, v_start, time_start, time_end - t1, v_max, a_max, ramp_ud_fragments))
      return false; // fail

  const double s0 = ramp_ud_fragments.empty() ? 0.0 : (ramp_ud_fragments.back().end - ramp_ud_fragments.front().start);
  const double end_check = start + s0 + s1;

  if (std::abs(end_check - end) > 10.0 * EPSILON)
    return false; // self consistency fail

  fragments.insert(fragments.end(), ramp_ud_fragments.begin(), ramp_ud_fragments.end());
  if (t1 != 0.0)
    fragments.push_back(JointTrajFrag(end - s1, end, v_start, v_end, -signed_a_max, time_end - t1, time_end, comment_accel0));

  // ensure that the last fragment ends exactly at end time
  if (!fragments.empty())
    fragments.back().time_end = time_end;

  return true;
}

inline double get_time_for_speed_trapezoid_sym(const double v_start_end,
                                               const double s, const double v_max, const double a_max)
{
  const double max_reachable_speed = std::sqrt(2.0 * a_max * (s / 2.0) + SQR(v_start_end));
  if (max_reachable_speed >= v_max) // saturate
  {
    const double s0 = (SQR(v_max) - SQR(v_start_end)) / (2.0 * a_max); // space during acceleration
    const double s1 = s - 2.0 * s0; // space at saturated speed
    const double t1 = s1 / v_max;
    const double t0 = (v_max - v_start_end) / a_max;
    const double t = t0 + t1 + t0;
    return t; // accel+const+decel
  }
  else // do not saturate
  {
    // twice the time to accelerate
    const double t = 2.0 * (max_reachable_speed - v_start_end) / a_max;
    return t;
  }
}

inline double get_time_for_speed_trapezoid(const double v_start, const double v_end_max,
                                           const double s, const double v_max, const double a_max)
{
  if (v_start > v_end_max) // if we are actually decelerating, invert v_end_max and v_start
    return get_time_for_speed_trapezoid(v_end_max, v_start, s, v_max, a_max);

  const double max_reachable_speed = std::sqrt(2.0 * a_max * s + SQR(v_start));
  if (max_reachable_speed >= v_end_max) // can reach end speed
  {
    const double t0 = (v_end_max - v_start) / a_max;
    const double s0 = v_start * t0  + 0.5 * a_max * SQR(t0);
    const double s1 = s - s0;
    const double t1 = (s1 > EPSILON) ? get_time_for_speed_trapezoid_sym(v_end_max, s1, v_max, a_max) : 0.0;
    const double t = t0 + t1;
    return t; // accel+trapezoid
  }
  else // cannot reach end speed, simply use constant acceleration
  {
    const double v_mean = (max_reachable_speed + v_start) / 2.0;
    const double t = s / v_mean;
    return t;
  }
}

inline double get_time_for_speed(const double v_start, const double s, const double v_max, const double a_max)
{
  if (v_start > v_max) // if we are actually decelerating, invert v_max and v_start
    return get_time_for_speed(v_max, s, v_start, a_max);

  const double max_reachable_speed = std::sqrt(2.0 * a_max * s + SQR(v_start));
  if (max_reachable_speed >= v_max) // saturate
  {
    const double s0 = (SQR(v_max) - SQR(v_start)) / (2.0 * a_max);
    const double t0 = (v_max - v_start) / a_max;
    const double s1 = s - s0;
    const double t1 = s1 / v_max;
    return t0 + t1;
  }
  else // do not saturate
  {
    const double v_mean = (max_reachable_speed + v_start) / 2.0;
    const double t = s / v_mean;
    return t;
  }
}

inline bool get_times_do_pass(const std::vector<Array6d> & trajectory, const double a_max,
                                const bool reverse, const Array6d & joint_speed_limits,
                                std::vector<Array6d> & velocities, std::vector<double> & times)
{
  const size_t size = trajectory.size();
  const int64_t start_i = reverse ? (size-2) : 1;
  const int64_t end_i = reverse ? -1 : size;

  bool changed = false;

  for (int64_t ii = start_i; ii != end_i; (reverse ? (ii--) : (ii++)))
  {
    const size_t i = ii;
    const size_t i_prev = (reverse ? (ii + 1) : (ii - 1));

    double max_time = 0.0;
    // find the time of the slowest joint
    for (size_t j = 0; j < NUM_JOINTS; j++)
    {
      const double s = std::abs(trajectory[i][j] - trajectory[i_prev][j]);
      if (s == 0.0)
        continue;
      const double prev_v = velocities[i_prev][j];
      const double v = velocities[i][j];
      const double my_v_max = joint_speed_limits[j] - EPSILON; // subtract epsilon to leave some margin for
      const double my_a_max = a_max - EPSILON;                 // numerical error
      double time;
      time = get_time_for_speed_trapezoid(prev_v, v, s, my_v_max, my_a_max); // using trapezoid
//    time = get_time_for_speed(prev_v, s, v, a_max);  // using constant acceleration + plateau

      if (max_time < time)
        max_time = time;
    }

    if (!reverse)
    {
      times[i] = std::max(max_time, times[i]);
      max_time = times[i];
    }
    else
    {
      times[i + 1] = std::max(max_time, times[i + 1]);
      max_time = times[i + 1];
    }

    if (max_time < EPSILON)
    {
      velocities[i] = velocities[i_prev];
      continue; // skip, nothing to do apparently
    }

    // set final velocities based on slowest joint
    for (size_t j = 0; j < NUM_JOINTS; j++)
    {
      const double s = std::abs(trajectory[i][j] - trajectory[i_prev][j]);
      const double prev_v = velocities[i_prev][j];
      const double v_mean = s / max_time;
      const double next_v = v_mean * 2.0 - prev_v;
      const double a = std::abs(next_v - prev_v) / max_time;
      if (next_v >= 0.0 && a <= (a_max + EPSILON)) // feasible
      {
        if (next_v < velocities[i][j])
        {
          velocities[i][j] = next_v;
          changed = true;
        }
      }
      else
      {
        if (v_mean < velocities[i][j]) // unfeasible, set next speed to maximum feasible,
        {                              //next iteration will take care of it
          velocities[i][j] = v_mean;
          changed = true;
        }
      }
    }
  }

  return changed;
}

inline std::vector<double> get_times(const std::vector<Array6d> & trajectory, const double a_max, const Array6d my_joint_speed_limits,
                                     std::vector<Array6d> & expected_velocities, uint64_t & iterations)
{
  const size_t size = trajectory.size();
  std::vector<Array6d> max_velocities(size, my_joint_speed_limits);

  std::vector<double> times(size, 0.0);

  // set the extremes to zero
  max_velocities[0] = ZEROS;
  max_velocities[size-1] = ZEROS;
  // set the inversion points to zero
  for (size_t i = 1; i < size - 1; i++)
  {
    for (size_t j = 0; j < NUM_JOINTS; j++)
    {
      double diff_prev = trajectory[i][j] - trajectory[i - 1][j];
      double diff_next = trajectory[i + 1][j] - trajectory[i][j];
      if (diff_prev * diff_next < 0)
        max_velocities[i][j] = 0.0;
    }
  }

  iterations = 0;

  uint64_t changed_counter = 2;
  bool reverse = false;
  while (changed_counter)
  {
    iterations++;

    const bool changed = get_times_do_pass(trajectory, a_max, reverse, my_joint_speed_limits, max_velocities, times);
    if (!changed)
      changed_counter--; // terminate if we can make two iterations without changes
    else
      changed_counter = 2;

    reverse = !reverse;
  }

  expected_velocities = max_velocities;

  // ensure velocities have correct signs
  for (size_t traj_i = 1; traj_i < size; traj_i++)
  {
    const Array6d p0 = trajectory[traj_i - 1];
    const Array6d p1 = trajectory[traj_i];

    for (size_t j = 0; j < NUM_JOINTS; j++)
    {
      if ((p1[j] - p0[j] < 0.0) != (expected_velocities[traj_i][j] < 0.0))
        expected_velocities[traj_i][j] = -expected_velocities[traj_i][j];
    }
  }

  return times;
}

inline std::vector<JointTrajSample> instantiate_joint_traj_frag(const JointTrajFrag fragment, const double dt)
{
  const double ceil_time_start = std::ceil(fragment.time_start / dt);
  const double floor_time_end = std::floor(fragment.time_end / dt);
  const int64_t first_fragment_i = int64_t(ceil_time_start);
  const int64_t last_fragment_i = (floor_time_end == fragment.time_end / dt) ?
                                  int64_t(floor_time_end) - 1 : // if ending exactly on a sample, then we do not sample it
                                  int64_t(floor_time_end);      // because it will be sampled by the next fragment

  const int64_t num_fragments = last_fragment_i - first_fragment_i + 1;
  std::vector<JointTrajSample> result(num_fragments);
  for (int64_t i = 0; i < num_fragments; i++)
  {
    const double global_t = double(first_fragment_i + i) * dt;
    const double t = global_t - fragment.time_start;
    double s = fragment.start + fragment.v_start * t + 0.5 * fragment.accel * SQR(t);
    JointTrajSample sample(s, global_t, fragment.comment);
    result[i] = sample;
  }

  return result;
}

struct TrajectoryFragments
{
  std::vector<JointTrajFrag> all_fragments[NUM_JOINTS];
};

inline
bool trajectory_fragments_planning(const std::vector<Array6d> & trajectory, const double a_max, const double v_max, std::vector<double> & times,
                                   std::vector<Array6d> & velocities, uint64_t & iterations, TrajectoryFragments & all_fragments)
{
  Array6d my_v_max(joint_speed_limits);
  for (double & v : my_v_max)
    v = std::min(v, v_max);

  times = get_times(trajectory, a_max, my_v_max, velocities, iterations);

  double prev_time = 0.0;
  for (size_t traj_i = 1; traj_i < trajectory.size(); traj_i++)
  {
    const Array6d p0 = trajectory[traj_i - 1];
    const Array6d p1 = trajectory[traj_i];
    const Array6d v0 = velocities[traj_i - 1];
    const Array6d v1 = velocities[traj_i];
    const double t0 = prev_time;
    const double t1 = prev_time + times[traj_i];
    prev_time = t1;

    for (size_t j = 0; j < NUM_JOINTS; j++)
    {
      const size_t NUM_PRIM = 6;
      // array of primitives
      const decltype(&build_traj_primitive_accel_plateau) primitives[NUM_PRIM] =
      {
        &build_traj_primitive_accel_plateau,
        &build_traj_primitive_plateau_accel,
        &build_traj_primitive_ramp_up_down,
        &build_traj_primitive_trapezoid,
        &build_traj_primitive_ramp_up_down_down,
        &build_traj_primitive_ramp_up_up_down
      };

      // try all known trajectory primitives
      std::vector<JointTrajFrag> fragments;
      bool found = false;
      for (size_t pi = 0; pi < NUM_PRIM && !found; pi++)
        if (primitives[pi](p0[j], p1[j], v0[j], v1[j], t0, t1, my_v_max[j], a_max, fragments))
          found = true;

      if (!found)
      {
        std::cout << "---- index " << traj_i << " joint " << j << " no success! ----" << std::endl;
        std::cout << "start " << p0[j] << " end " << p1[j] << " v_start " << v0[j] << " v_end " << v1[j] <<
                     " time_start " << t0 << " time_end " << t1 << " v_max " << my_v_max[j] << " a_max " << a_max << std::endl;
        std::cout << "----" << std::endl;
        return false;
      }

      // update the comment
      for (JointTrajFrag & jtf : fragments)
        jtf.comment = jtf.comment->withWaypoint(traj_i)->withJoint(j);

      all_fragments.all_fragments[j].insert(all_fragments.all_fragments[j].end(), fragments.begin(), fragments.end());
    }
  }

  return true;
}

// this actually returns the size, i.e. one past maximum index
inline
size_t trajectory_get_num_trajectory_samples(const TrajectoryFragments & fragments, const double dt)
{
  if (fragments.all_fragments[0].empty())
    return 0; // empty traj -> no samples

  const double time_end = fragments.all_fragments[0].back().time_end;
  size_t end_index = size_t(std::ceil(time_end / dt)) + 1; // +1 to add the end state

  return end_index;
}

inline
void trajectory_realign_fragment_indices(const TrajectoryFragments & fragments, const size_t time_index,
                                         Array6size & indices_cache, const double dt)
{
  const double time = double(time_index) * dt;
  for (size_t j = 0; j < NUM_JOINTS; j++)
  {
    // for each joint, find current fragment
    while (true)
    {
      if (indices_cache[j] >= fragments.all_fragments[j].size())
      {
        // if we are past the end, check if we must go back to the last fragment
        if (!fragments.all_fragments[j].empty() && time < fragments.all_fragments[j].back().time_end)
        {
          indices_cache[j] = fragments.all_fragments[j].size() - 1;
          continue;
        }
        else
        {
          break; // else, just exit
        }
      }

      const JointTrajFrag & fragment = fragments.all_fragments[j][indices_cache[j]];

      // check that we are still in this fragment
      if (time < fragment.time_start)
      {
        if (indices_cache[j] == 0)
          break; // cannot go back, break

        indices_cache[j]--;
      }
      else if (time >= fragment.time_end) // = because if the requested time
                                          // is exactly at the end time, it belongs to the next fragment
      {
        if (indices_cache[j] >= fragments.all_fragments[j].size())
          break; // cannot go forward, break

        indices_cache[j]++;
      }
      else
        break; // current fragment is ok
    }
  }
}

inline
TrajectorySample trajectory_extract_sample(const TrajectoryFragments & fragments, const size_t time_index,
                                           Array6size & indices_cache, const double dt)
{
  trajectory_realign_fragment_indices(fragments, time_index, indices_cache, dt);

  typedef JointTrajFrag::Comment Comment;
  typedef JointTrajFrag::Primitive Primitive;

  // pre-compute one end comment for each joint
  // unscrupulous abuse of a lambda is happening here
  static const std::vector<std::shared_ptr<const Comment> > end_comments = []() {
    std::shared_ptr<const Comment> end_comment = Comment().withPrimitive(Primitive::END);
    std::vector<std::shared_ptr<const Comment> > result(NUM_JOINTS);
    for (size_t j = 0; j < NUM_JOINTS; j++)
      result[j] = end_comment->withJoint(j);
    return result;
  }();

  TrajectorySample result;
  const double global_t = time_index * dt;
  result.t = global_t;
  for (size_t j = 0; j < NUM_JOINTS; j++)
  {
    if (indices_cache[j] >= fragments.all_fragments[j].size()) // past the end of traj for this joint
    {
      result.p[j] = fragments.all_fragments[j].back().end;
      result.v[j] = fragments.all_fragments[j].back().v_end;
      result.comments[j] = end_comments[j];
    }
    else
    {
      const JointTrajFrag & fragment = fragments.all_fragments[j][indices_cache[j]];
      const double t = global_t - fragment.time_start;
      double s = fragment.start + fragment.v_start * t + 0.5 * fragment.accel * SQR(t);
      double v = fragment.v_start + fragment.accel * t;
      result.p[j] = s;
      result.v[j] = v;
      result.comments[j] = fragment.comment;
    }
  }

  return result;
}

inline Array6size trajectory_create_cache()
{
  return ZERO_SIZES;
}

inline
bool trajectory_time_planning(const std::vector<Array6d> & trajectory, const double a_max, const double v_max, const double dt,
                              std::vector<double> & times,
                              std::vector<Array6d> & velocities, uint64_t & iterations, std::vector<TrajectorySample> & samples)
{
  TrajectoryFragments fragments;
  if (!trajectory_fragments_planning(trajectory, a_max, v_max, times, velocities, iterations, fragments)){
    std::cout << "Terminating execution, no trajectory fragments generated" << std::endl;
    return false;
  }

  Array6size indices_cache = trajectory_create_cache();
  const size_t max_time_index = trajectory_get_num_trajectory_samples(fragments, dt);

  if (max_time_index*dt > 24.0f * 3600.0f) {
    std::cout << "Terminating execution, trajectory duration is longer than a day! (sanity check)" << std::endl;
    return false;
  }

  for (size_t i = 0; i < max_time_index; i++)
  {
    TrajectorySample sample = trajectory_extract_sample(fragments, i, indices_cache, dt);
    samples.push_back(sample);
  }

  return true;
}

#endif // TRAJECTORY_TIME_PLANNING_H
