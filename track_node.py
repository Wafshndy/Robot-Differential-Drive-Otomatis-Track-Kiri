import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


def norm(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, maximum):
    return max(-maximum, min(maximum, value))


class RobotTrackNode(Node):
    def __init__(self):
        super().__init__('geraklintasan')

        self.publisher_ = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        self.sub = self.create_subscription(
            Float64MultiArray,
            '/wheel/speed',
            self.encoder,
            1
        )

        # T: target arah absolut, derajat.
        # G: target x, target y, kecepatan fisik m/s.
        # A: pusat x, pusat y, radius, sudut busur, m/s.
        # W: waktu berhenti, detik.
        #
        # Koordinat dihitung relatif terhadap spawn:
        # x = 0, y = 0, theta = 0.

        self.plan = [
            ('T', 90),
            ('G', 0.00, 2.38, 0.30),

            ('T', 0),
            ('G', 0.93, 2.38, 0.24),

            ('W', 4.00),

            ('G', 2.37, 2.38, 0.25),

            ('T', 90),
            ('G', 2.37, 3.04, 0.22),

            ('T', 0),
            ('G', 3.66, 3.04, 0.25),

            ('T', -90),
            ('G', 3.66, 0.79, 0.34),

            ('T', 0),
            ('G', 7.91, 0.79, 0.38),

            ('T', 90),
            ('G', 7.91, 5.82, 0.38),

            # Pusat X, pusat Y, radius, sudut, kecepatan
            ('A', 6.38, 5.82, 1.53, 180, 0.26),

            ('G', 4.85, 4.27, 0.28),

            ('T', 180),
            ('G', 3.94, 4.27, 0.23),

            ('T', 90),
            ('G', 3.94, 6.78, 0.34),

            ('T', 180),
            ('G', 3.16, 6.78, 0.22),

            ('T', 90),
            ('G', 3.16, 7.54, 0.22),

            ('T', 180),
            ('G', 0.28, 7.54, 0.35),

            ('T', -90),
            ('G', 0.28, 5.00, 0.30),
        ]

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.v = 0.0
        self.w = 0.0
        self.wout = 0.0

        self.feedback = None
        self.index = 0

        self.active = False
        self.arrived = False
        self.failed = False
        self.finished = False
        self.enabled = False

        self.ready_since = None
        self.settled_since = None
        self.started = None

        self.tick = time.monotonic()

        self.timer = self.create_timer(
            0.02, self.control_loop
        )

        self.get_logger().info(
            'Reset ke spawn awal. '
            'Menunggu feedback roda dan kondisi diam 2 detik.'
        )

    def send(self, v, w):
        msg = Twist()

        # v dan w di pengendali adalah kecepatan fisik.
        msg.linear.x = float(v / 0.30)
        msg.angular.z = float(w / 0.7941176471)

        self.publisher_.publish(msg)

    def fail(self, message):
        if not self.failed:
            self.failed = True
            self.get_logger().error(
                message + ' STOP; reset sebelum ulang.'
            )

        self.send(0.0, 0.0)

    def encoder(self, msg):
        now = time.monotonic()

        if (
            len(msg.data) != 2
            or not all(math.isfinite(a) for a in msg.data)
        ):
            self.fail('Data roda tidak valid.')
            return

        left, right = msg.data

        # Forward kinematics:
        # radius roda = 0.03 m
        # jarak antarroda = 0.17 m
        v = 0.03 * (left + right) / 2.0
        w = 0.03 * (right - left) / 0.17

        if (
            self.enabled
            and not self.failed
            and self.feedback is not None
        ):
            dt = now - self.feedback

            if dt > 0.30:
                self.fail('Feedback roda terputus.')

            elif dt > 0.0:
                # Rata-rata kecepatan.
                vb = (self.v + v) / 2.0
                omega = (self.w + w) / 2.0

                # Estimasi posisi global dengan midpoint.
                mid = self.theta + omega * dt / 2.0

                self.x += vb * math.cos(mid) * dt
                self.y += vb * math.sin(mid) * dt
                self.theta = norm(self.theta + omega * dt)

        self.v = v
        self.w = w
        self.feedback = now

    def control_loop(self):
        now = time.monotonic()
        dt = now - self.tick
        self.tick = now

        if self.failed or self.finished:
            self.send(0.0, 0.0)
            return

        if self.count_publishers('/cmd_vel') > 1:
            self.fail('Ada node penggerak lain.')
            return

        # Mulai setelah feedback tersedia.
        if not self.enabled:
            self.send(0.0, 0.0)

            ready = (
                self.feedback is not None
                and now - self.feedback < 0.15
                and abs(self.v) < 0.01
                and abs(self.w) < 0.03
                and self.publisher_.get_subscription_count() > 0
            )

            if not ready:
                self.ready_since = None

            elif self.ready_since is None:
                self.ready_since = now

            elif now - self.ready_since >= 2.0:
                self.enabled = True

            return

        if dt > 0.15 or now - self.feedback > 0.30:
            self.fail('Kontrol atau feedback terlambat.')
            return

        if (
            self.started is not None
            and now - self.started > 300.0
        ):
            self.fail('Belum selesai dalam 5 menit.')
            return

        if self.index == len(self.plan):
            self.finished = True
            self.send(0.0, 0.0)

            elapsed = now - self.started

            self.get_logger().info(
                f'SELESAI MENURUT ODOMETRI | '
                f'RAW TIME: {elapsed:.2f} detik '
                f'({elapsed / 60:.2f} menit), '
                'termasuk checkpoint. '
                'Konfirmasi kotak finish secara visual.'
            )
            return

        step = self.plan[self.index]
        kind = step[0]

        # Inisialisasi tahap baru.
        if not self.active:
            self.active = True
            self.arrived = False
            self.settled_since = None

            self.action_start = now
            self.swept = 0.0

            if kind == 'A':
                self.previous_alpha = math.atan2(
                    self.y - step[2],
                    self.x - step[1]
                )

            self.get_logger().info(
                f'Tahap {self.index + 1}: {step}'
            )

        v = 0.0
        w = 0.0

        # Checkpoint dimulai setelah tahap sebelumnya
        # selesai dan roda sudah berhenti.
        if kind == 'W':
            self.send(0.0, 0.0)

            if now - self.action_start >= step[1]:
                self.index += 1
                self.active = False

            return

        if not self.arrived:

            # Putar menuju arah absolut.
            if kind == 'T':
                error = norm(
                    math.radians(step[1]) - self.theta
                )

                self.arrived = abs(error) < 0.012
                w = 3.0 * error

            # Bergerak menuju target koordinat.
            elif kind == 'G':
                dx = step[1] - self.x
                dy = step[2] - self.y

                distance = math.hypot(dx, dy)
                target_heading = math.atan2(dy, dx)

                error = norm(target_heading - self.theta)
                self.arrived = distance <= 0.02

                # Cruise konstan, lalu mengerem.
                # v_rem = sqrt(2 * a * s), a = 0.30 m/s².
                v = min(
                    step[3],
                    math.sqrt(
                        0.60 * max(distance - 0.008, 0.0)
                    )
                )

                # Kurangi maju ketika arah menyimpang.
                v *= max(0.0, math.cos(error))

                if abs(error) > math.radians(20):
                    v = 0.0

                w = clamp(3.0 * error, 0.50)

            # Mengikuti busur lingkaran.
            else:
                _, cx, cy, radius, angle, speed = step

                sign = 1 if angle > 0 else -1

                alpha = math.atan2(
                    self.y - cy,
                    self.x - cx
                )

                self.swept += norm(
                    alpha - self.previous_alpha
                )
                self.previous_alpha = alpha

                remaining = (
                    abs(math.radians(angle))
                    - sign * self.swept
                )

                self.arrived = remaining <= 0.008

                radial_error = (
                    radius - math.hypot(
                        self.x - cx,
                        self.y - cy
                    )
                )

                desired = (
                    alpha
                    + sign * math.pi / 2.0
                    - sign * 1.5 * radial_error
                )

                error = norm(desired - self.theta)

                v = min(
                    speed,
                    math.sqrt(
                        0.60 * max(
                            radius * remaining - 0.008,
                            0.0
                        )
                    )
                )

                v *= max(0.0, math.cos(error))

                # Feedforward lingkaran + koreksi arah.
                w = sign * v / radius + 2.5 * error

        if self.arrived:
            v = 0.0
            w = 0.0

        # Batasi perubahan putaran agar ga nyentak.
        # Kecepatan lurus langsung menuju setpoint.
        self.wout += clamp(
            clamp(w, 0.80) - self.wout,
            1.6 * dt
        )

        self.send(v, self.wout)

        # Mulai menghitung stopwatch saat tahap 6 bergerak masuk track.
        # Indeks Python dimulai dari 0, jadi tahap 6 = indeks 5.
        if (
            self.started is None
            and self.index == 5
            and v > 1e-8
        ):
            self.started = now

            self.get_logger().info(
                'MASUK TRACK — TIMER DIMULAI'
            )

        stopped = (
            abs(self.v) < 0.02
            and abs(self.w) < 0.05
            and abs(self.wout) < 0.01
        )

        # Pastikan robot berhenti sebelum tahap berikutnya.
        if self.arrived and stopped:
            if self.settled_since is None:
                self.settled_since = now

            elif now - self.settled_since >= 0.10:
                self.index += 1
                self.active = False
                self.wout = 0.0

        else:
            self.settled_since = None

        # Kecepatan dalam koordinat global.
        vx = self.v * math.cos(self.theta)
        vy = self.v * math.sin(self.theta)

        self.get_logger().info(
            f'estimasi x={self.x:.2f}, y={self.y:.2f}, '
            f'theta={math.degrees(self.theta):.1f} deg | '
            f'vx={vx:.2f}, vy={vy:.2f} m/s',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = RobotTrackNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if rclpy.ok():
            for _ in range(5):
                node.send(0.0, 0.0)
                time.sleep(0.02)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
