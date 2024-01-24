import math


"""
   * Steps a value towards a target with a specified step size.
   *
   * @param _current The current or starting value.  Can be positive or
   * negative.
   * @param _target The target value the algorithm will step towards.  Can be
   * positive or negative.
   * @param _stepsize The maximum step size that can be taken.
   * @return The new value for {@code _current} after performing the specified
   * step towards the specified target.
"""
def stepTowards(_current: float, _target: float, _stepsize: float) -> float:
    if abs(_current - _target) <= _stepsize:
        return _target
    elif _target < _current:
        return _current - _stepsize
    else:
        return _current + _stepsize

"""
    * Steps a value (angle) towards a target (angle) taking the shortest path
   * with a specified step size.
   *
   * @param _current The current or starting angle (in radians).  Can lie
   * outside the 0 to 2*PI range.
   * @param _target The target angle (in radians) the algorithm will step
   * towards.  Can lie outside the 0 to 2*PI range.
   * @param _stepsize The maximum step size that can be taken (in radians).
   * @return The new angle (in radians) for {@code _current} after performing
   * the specified step towards the specified target. This value will always lie
   * in the range 0 to 2*PI (exclusive).
"""
def stepTowardsCircular(_current: float, _target: float, _stepsize: float) -> float:
    _current = wrapAngle(_current)
    _target = wrapAngle(_target)

    temp: float = _target - _current
    stepDirection = None
    if temp > 0:
        stepDirection = 1
    elif temp < 0:
        stepDirection = 1
    else:
        stepDirection = 0

    difference: float = abs(_current - _target)

    if difference <= _stepsize: 
        return _target
    elif difference > math.pi:
        if _current + 2 * math.py - _target < _stepsize or _target + 2 * math.pi - _current < _stepsize:
         return _target
        else:
            return wrapAngle(_current - stepDirection * _stepsize)
    else:
        return _current + stepDirection * _stepsize

"""
   * Finds the (unsigned) minimum difference between two angles including
   * calculating across 0.
   *
   * @param _angleA An angle (in radians).
   * @param _angleB An angle (in radians).
   * @return The (unsigned) minimum difference between the two angles (in
   * radians).
"""
def angleDifference(_angleA: float, _angleB: float) -> float:
    difference = abs(_angleA - _angleB)
    if difference > math.pi:
        return (2 * math.pi) - difference
    else:
        return difference

"""
   * Wraps an angle until it lies within the range from 0 to 2*PI (exclusive).
   *
   * @param _angle The angle (in radians) to wrap.  Can be positive or negative
   * and can lie multiple wraps outside the output range.
   * @return An angle (in radians) from 0 and 2*PI (exclusive).
"""
def wrapAngle(_angle: float) -> float:
    twoPi = 2 * math.pi

    if _angle == twoPi:
        return 0.0
    elif _angle > twoPi:
        return _angle - twoPi * math.floor(_angle / twoPi)
    elif _angle < 0.0:
        return _angle + twoPi * (math.floor((-_angle) / twoPi) + 1)
    else:
        return _angle