class PID:
  def __init__(self, kp, ki, kd, lower=None, upper=None, **kwargs):
    self._kp = float(kp)
    self._ki = float(ki)
    self._kd = float(kd)
    self._lower = lower
    self._upper = upper
    self._e = 0.0
    self._e_prev = 0.0
    self._e_int = 0.0
    self._de = 0.0
    self._u = 0.0
  
  def update(self, y, w, dt):
    self._e_prev = self._e
    self._e = w - y
    self._e_int = self._e_int + dt * self._e
    self._de = (self._e - self._e_prev) / dt
    self._u = self._kp * self._e + self._ki * self._e_int + self._kd * self._de
    if self._lower is not None and self._u < self._lower:
      self._u = self._lower
    if self._upper is not None and self._u > self._upper:
      self._u = self._upper
    return self._u
  
  def reset(self):
    self._e_int = 0.0
    self._e_prev = self._e

