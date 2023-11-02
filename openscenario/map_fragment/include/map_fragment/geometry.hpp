// Copyright 2015 TIER IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MAP_FRAGMENT__GEOMETRY__HPP_
#define MAP_FRAGMENT__GEOMETRY__HPP_

#include <cmath>
#include <string>
#include <algorithm>
#include <memory>
#include <stdexcept>

namespace map_fragment
{

struct Point2d
{
  double x = 0;
  double y = 0;
};

struct Vector2d
{
  double x = 0;
  double y = 0;
};

Point2d operator+(Point2d a, Vector2d const& b)
{
  a.x += b.x;
  a.y += b.y;
  return a;
}

Vector2d operator*(Vector2d v, double const& multiplier)
{
  v.x *= multiplier;
  v.y *= multiplier;
  return v;
}

Vector2d operator-(Point2d const& a, Point2d const& b)
{
  return {a.x - b.x, a.y - b.y};
}

Vector2d operator+(Vector2d const& a, Vector2d const& b)
{
  return {a.x + b.x, a.y + b.y};
}

Vector2d rotate(Vector2d v, double angle) {
  return {
    v.x * std::cos(angle) - v.y * std::sin(angle),
    v.x * std::sin(angle) + v.y * std::cos(angle)
  };
}

Point2d rotate(Point2d p, double angle)
{
  Point2d p0 = {0, 0};
  return p0 + rotate(p - p0, angle);
}

double calculateAngle(Vector2d v)
{
  return std::atan2(v.y, v.x);
}

struct Transformation2d
{
  Vector2d translation;
  double rotation;
};  // struct Transformation2d

Point2d applyTransformation(Point2d p, Transformation2d t)
{
  return rotate(p, t.rotation) + t.translation;
}

Vector2d applyTransformation(Vector2d v, Transformation2d t)
{
  return rotate(v, t.rotation);
}

Transformation2d chainTransformations(Transformation2d first, Transformation2d second)
{
  Transformation2d t;
  t.translation = first.translation + rotate(second.translation, first.rotation);
  t.rotation = first.rotation + second.rotation;
  return t;
}

/**
 * Parametric representation of a smooth curve in 2D Cartesian space
 * 
 * By convention, the curve is assumed to always start at point (0, 0)
 * and have a tangent vector there of [1, 0]. Geometric trasformations
 * can be applied afterwards if other start positions and/or
 * directions are required.
 */
class ParametricCurve
{

public:
  using Ptr = std::shared_ptr<ParametricCurve>;
  
  /**
   * Calculate curve point for given parameter t ∈ [0, 1]
   */
  Point2d getPosition(double t)
  {
    validateCurveParameterOrThrow(t);
    return getPosition_(t);
  }

  /**
   * Calculate curve tangent vector for given parameter t ∈ [0, 1]
   */
  Vector2d getTangentVector(double t)
  {
    validateCurveParameterOrThrow(t);

    // TODO Throw error if the vector is not unit?
    return getTangentVector_(t);
  }

private:
  virtual Point2d getPosition_(double t) = 0;
  virtual Vector2d getTangentVector_(double t) = 0;

protected:
  void validateCurveParameterOrThrow(double t)
  {
    if (!(0 <= t && t <= 1)) {
      throw std::invalid_argument(
        "Expected t to be in range [0, 1]. Actual value: "
        + std::to_string(t)
      );
    }
  }
};  // class ParametricCurve

/**
 * Straight line of given length
 */
class Straight : public ParametricCurve
{
  double length_;

public:
  Straight(double length)
    : length_(length)
  {
    if (length <= 0.0) {
      throw std::invalid_argument(
        "Expected length to be positive. Actual value: "
        + std::to_string(length)
      );
    }
  }

private:
  virtual Point2d getPosition_(double t) override
  {
    return {t * length_, 0};
  }
  
  virtual Vector2d getTangentVector_(double) override
  {
    return {1, 0};
  }
};  // class Straight

/**
 * Arc of given radius and angle (positive to the left)
 */
class Arc : public ParametricCurve
{
  double radius_;
  double angle_;
  Point2d center_;

public:
  Arc(double radius, double angle)
    : radius_(radius), angle_(angle)
  {
    if (radius <= 0.0)
    {
      throw std::invalid_argument(
        "Expected radius to be positive. Actual value:"
        + std::to_string(radius)
      );
    }

    if (!(0 < abs(angle) && abs(angle) < 2 * M_PI))
    {
      throw std::invalid_argument(
        "Expected angle to be in range (0, 2 * PI) or (-2 * PI, 0). Actual value: "
        + std::to_string(angle)
      );
    }

    center_.y = angle > 0 ? radius : -radius;
  }
  
private:
  virtual Point2d getPosition_(double t) override
  {
    auto theta = t * angle_;
    Point2d origin = {0, 0};
    return center_ + rotate(origin - center_, theta);
  }
  
  virtual Vector2d getTangentVector_(double t) override
  {
    auto theta = t * angle_;
    Vector2d v = {1, 0};
    return rotate(v, theta);
  }
};  // class Arc

/**
 * Combination of two or more component curves, chained after one another
 */
class CombinedCurve : public ParametricCurve
{
  std::vector<ParametricCurve::Ptr> curves_;
  std::vector<Transformation2d> transformations_;

public:
  CombinedCurve(std::vector<ParametricCurve::Ptr> curves)
    : curves_(curves)
  {
    if (curves.size() < 2)
    {
      throw std::invalid_argument(
        "Number of curves to be combined must be two or more. Actual number: "
        + std::to_string(curves.size())
      );
    }

    Transformation2d current_transformation;

    for (const auto& curve : curves)
    {
      transformations_.push_back(current_transformation);
      
      Point2d origin = {0, 0};
      Transformation2d local_transformation = {
        curve->getPosition(1.0) - origin,
        calculateAngle(curve->getTangentVector(1.0))
      };

      current_transformation = chainTransformations(current_transformation,
                                                    local_transformation);
    }
  }

private:
  virtual Point2d getPosition_(double t) override
  {
    auto [curve_id, curve_t] = getCurveIdAndParameter(t);
    auto curve = curves_[curve_id];
    auto transformation = transformations_[curve_id];

    return applyTransformation(curve->getPosition(curve_t), transformation);
  }

  virtual Vector2d getTangentVector_(double t) override
  {
    auto [curve_id, curve_t] = getCurveIdAndParameter(t);
    auto curve = curves_[curve_id];
    auto transformation = transformations_[curve_id];

    return applyTransformation(curve->getTangentVector(curve_t), transformation);
  }

  std::pair<unsigned long, double> getCurveIdAndParameter(double t) {
    validateCurveParameterOrThrow(t);
    
    auto range_width_per_curve = 1. / curves_.size();
    auto curve_id = std::min(
      static_cast<unsigned long>(t / range_width_per_curve),
      curves_.size() - 1);
    auto curve_t = t / range_width_per_curve - curve_id;

    return std::make_pair(curve_id, curve_t);
  }
};  // class CombinedCurve

/**
 * Geometric transformation applied to a base curve.
 */
class TransformedCurve : public ParametricCurve
{
  const ParametricCurve::Ptr base_curve_;
  const Transformation2d transformation_;

public:
  TransformedCurve(const ParametricCurve::Ptr& base_curve,
                   const Transformation2d& transformation)
    : base_curve_(base_curve)
    , transformation_(transformation)
  { }

private:
  virtual Point2d getPosition_(double t) override
  {
    return applyTransformation(base_curve_->getPosition(t),
                               transformation_);
  }

  virtual Vector2d getTangentVector_(double t) override
  {
    return applyTransformation(base_curve_->getTangentVector(t),
                               transformation_);
  }
};  // class TransformedCurve

/**
 * Lateral shift applied to a base curve.
 */

class LaterallyShiftedCurve : public ParametricCurve
{
  const ParametricCurve::Ptr base_curve_;
  const double lateral_shift_;

public:
  LaterallyShiftedCurve(const ParametricCurve::Ptr base_curve,
                        const double lateral_shift)
    : base_curve_(base_curve)
    , lateral_shift_(lateral_shift)
  { }

private:
  virtual Point2d getPosition_(double t) override
  {
    auto base_position = base_curve_->getPosition(t);
    auto normal_vector = rotate(base_curve_->getTangentVector(t), M_PI / 2);

    return base_position + normal_vector * lateral_shift_;
  }

  virtual Vector2d getTangentVector_(double t) override
  {
    return base_curve_->getTangentVector(t);
  }
};  // class LaterallyShiftedCurve

void transformCurve(const ParametricCurve::Ptr& curve_in,
                    const Transformation2d& transformation,
                    ParametricCurve::Ptr& curve_out)
{
  curve_out = std::make_shared<TransformedCurve>(curve_in, transformation);
}

void shiftCurveLaterally(const ParametricCurve::Ptr& curve_in,
                         const double& lateral_offset,
                         ParametricCurve::Ptr& curve_out)
{
  curve_out = std::make_shared<LaterallyShiftedCurve>(curve_in, lateral_offset);
}

}  // namespace map_fragment

#endif  // MAP_FRAGMENT__GEOMETRY__HPP_