"""Closed-course wall layouts and directional timing gates, in meters."""

from dataclasses import dataclass
import math

from .conditions import GripZone


def unit_direction(start, end):
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = math.hypot(dx, dy)
    return dx / length, dy / length


def track_outline(vertices):
    """Offset the two walls by the specified widths, with mitered corners."""
    outer, inner = [], []
    for index, ((x, y), outside, inside) in enumerate(vertices):
        ax, ay = unit_direction(vertices[index - 1][0], (x, y))
        bx, by = unit_direction((x, y), vertices[(index + 1) % len(vertices)][0])
        miter = 1 + ax * bx + ay * by
        nx, ny = -(ay + by) / miter, (ax + bx) / miter
        outer.append((x - outside * nx, y - outside * ny))
        inner.append((x + inside * nx, y + inside * ny))
    return tuple(outer), tuple(inner)


@dataclass(frozen=True)
class Checkpoint:
    label: str
    center: tuple[float, float]
    direction: tuple[float, float]
    detail: str
    half_width: float = 0.85
    display_width: float = 0.33  # Half-length of the line drawn for the original tracks.

    def coordinates(self, point):
        x, y = point[0] - self.center[0], point[1] - self.center[1]
        dx, dy = self.direction
        return x * dx + y * dy, -x * dy + y * dx

    def crossed(self, start, end):
        along_a, side_a = self.coordinates(start)
        along_b, side_b = self.coordinates(end)
        if not along_a < 0 <= along_b:
            return False
        side = (along_b * side_a - along_a * side_b) / (along_b - along_a)
        return abs(side) <= self.half_width


@dataclass(frozen=True)
class Scenario:
    label: str
    outlines: tuple
    start_y: float = 0.0
    missing_walls: tuple = ()
    checkpoints: tuple[Checkpoint, ...] = ()
    grip_zones: tuple[GripZone, ...] = ()


def edge(x, y, outside, inside=None):
    return ((x, y), outside, outside if inside is None else inside)


def arc(x, y, radius, start, end, half_width):
    segments = math.ceil((end - start) / 15)
    angles = (math.radians(start + (end - start) * i / segments) for i in range(segments + 1))
    return tuple(edge(x + radius * math.cos(a), y + radius * math.sin(a), half_width) for a in angles)


def gate(label, x, y, heading, half_width, detail):
    bearing = math.radians(heading)
    return Checkpoint(label, (x, y), (math.cos(bearing), math.sin(bearing)), detail,
                      half_width, half_width)


HAIRPIN_RADIUS = 0.46
HAIRPIN_HALF_WIDTH = 0.35
HAIRPIN = (arc(6.0, HAIRPIN_RADIUS, HAIRPIN_RADIUS, -90, 90, HAIRPIN_HALF_WIDTH)
           + arc(-2.0, HAIRPIN_RADIUS, HAIRPIN_RADIUS, 90, 270, HAIRPIN_HALF_WIDTH))
HAIRPIN_GATES = (
    gate("헤어핀 진입", 4.8, 0.0, 0, 0.35, "긴 직선에서 감속"),
    gate("동쪽 180° 코너 정점", 6.46, 0.46, 90, 0.35, "중심선 반경 0.46 m"),
    gate("동쪽 헤어핀 탈출", 5.2, 0.92, 180, 0.35, "반대 방향 직선으로 가속"),
    gate("서쪽 헤어핀 진입", -1.2, 0.92, 180, 0.35, "평행 직선에서 감속"),
    gate("서쪽 180° 코너 정점", -2.46, 0.46, 270, 0.35, "폭 0.70 m"),
    gate("서쪽 헤어핀 탈출", -1.2, 0.0, 0, 0.35, "정점 이후 조향 복귀"),
    gate("결승선", 0, 0, 0, 0.35, "두 헤어핀 통과"),
)

NARROW = ((edge(3.6, 0, 0.48), edge(4.5, 0, 0.34))
          + arc(5.0, 0.60, 0.60, -90, 0, 0.28)
          + (edge(5.60, 1.35, 0.28), edge(5.60, 2.0, 0.41))
          + arc(5.0, 3.0, 0.60, 0, 90, 0.48)
          + arc(-1.7, 3.0, 0.60, 90, 180, 0.48)
          + arc(-1.7, 0.60, 0.60, 180, 270, 0.48))
NARROW_GATES = (
    gate("협로 진입", 4.0, 0, 0, 0.42, "폭 0.96 m에서 점차 좁아짐"),
    gate("좁은 90° 코너 탈출", 5.60, 1.20, 90, 0.28, "코너 폭 0.56 m"),
    gate("다시 넓어지는 통로", 5.60, 2.4, 90, 0.42, "출구 가속"),
    gate("북쪽 직선", 3.5, 3.6, 180, 0.48, "직선 속도 비교"),
    gate("서쪽 코너 진입", -1.35, 3.6, 180, 0.48, "폭 0.96 m 코너"),
    gate("서쪽 직선", -2.30, 1.6, 270, 0.48, "첫 코너와 벽 여유 비교"),
    gate("결승선", 0, 0, 0, 0.48, "좁은 코너 포함 한 바퀴"),
)


def s_point(fraction):
    phase = math.tau * fraction
    return 1.6 + 5.0 * fraction, 0.96 * math.sin(phase / 2) ** 2


def s_gate(label, fraction, detail):
    slope = 0.96 * math.pi / 5.0 * math.sin(math.tau * fraction)
    return gate(label, *s_point(fraction), math.degrees(math.atan(slope)), 0.38, detail)


S_CURVE = (tuple(edge(*s_point(index / 20), 0.38) for index in range(21))
           + arc(7.6, 0.75, 0.75, -90, 0, 0.48)
           + arc(7.6, 3.4, 0.75, 0, 90, 0.48)
           + arc(-1.65, 3.4, 0.75, 90, 180, 0.48)
           + arc(-1.65, 0.75, 0.75, 180, 270, 0.48))
S_GATES = (
    gate("S자 진입", 1.6, 0, 0, 0.38, "폭 0.76 m"),
    s_gate("첫 방향 전환", 0.25, "정면에서 좌우 벽 방향이 변화"),
    s_gate("S자 중앙", 0.50, "중심선 옆 이동 최대 0.96 m"),
    s_gate("두 번째 방향 전환", 0.75, "반대 방향으로 다시 조향"),
    gate("S자 탈출", 6.6, 0, 0, 0.38, "중심선이 직선으로 복귀"),
    gate("복귀 직선", 3.4, 4.15, 180, 0.48, "S자와 직선의 속도 비교"),
    gate("결승선", 0, 0, 0, 0.42, "S자 포함 한 바퀴"),
)

GAP_START_X, GAP_END_X = 4.05, 3.83
OFFSET = ((edge(1.0, 0, 0.48), edge(2.1, 0, 0.25, 0.51), edge(2.9, 0, 0.25, 0.51),
           edge(4.0, 0, 0.51, 0.25), edge(4.8, 0, 0.51, 0.25))
          + arc(6.2, 0.65, 0.65, -90, 0, 0.46)
          + arc(6.2, 2.7, 0.65, 0, 90, 0.46)
          + (edge(GAP_START_X, 3.35, 0.46), edge(GAP_END_X, 3.35, 0.46))
          + arc(-1.7, 2.7, 0.65, 90, 180, 0.46)
          + arc(-1.7, 0.65, 0.65, 180, 270, 0.46))
OFFSET_OUTLINES = track_outline(OFFSET)
OFFSET_GAP = next((a, b) for a, b in zip(OFFSET_OUTLINES[1], OFFSET_OUTLINES[1][1:])
                  if math.isclose(a[0], GAP_START_X) and math.isclose(b[0], GAP_END_X))
OFFSET_GATES = (
    gate("한쪽 벽 돌출", 2.65, 0.13, 0, 0.38, "첫 협로 중심이 한쪽으로 이동"),
    gate("반대쪽 벽 돌출", 4.65, -0.13, 0, 0.38, "협로 중심이 반대쪽으로 이동"),
    gate("동쪽 90° 탈출", 6.85, 1.6, 90, 0.46, "엇갈린 협로 다음 코너"),
    gate("한쪽 벽 단절", (GAP_START_X + GAP_END_X) / 2, 3.35, 180, 0.46, "안쪽 벽 0.22 m 누락"),
    gate("서쪽 코너 진입", -1.15, 3.35, 180, 0.46, "벽 단절 후 직선 복귀"),
    gate("서쪽 직선", -2.35, 1.5, 270, 0.46, "코너 감속과 차체 여유 확인"),
    gate("결승선", 0, 0, 0, 0.46, "엇갈린 벽과 단절 포함 한 바퀴"),
)

CHALLENGE_COURSES = {
    "loop-hairpin": Scenario("헤어핀 순환 · 반경 0.46 m", track_outline(HAIRPIN), checkpoints=HAIRPIN_GATES,
                             grip_zones=(GripZone(5.7, 6.84, -0.36, 1.27, 0.68),)),
    "loop-narrow": Scenario("협로 순환 · 코너 폭 0.56 m", track_outline(NARROW), checkpoints=NARROW_GATES,
                            grip_zones=(GripZone(4.6, 5.92, -0.34, 1.35, 0.68),)),
    "loop-s": Scenario("S자 순환 · 연속 방향 전환", track_outline(S_CURVE), checkpoints=S_GATES,
                       grip_zones=(GripZone(3.2, 5.05, 0.0, 1.47, 0.68),)),
    "loop-offset": Scenario("비대칭 순환 · 엇갈린 벽과 단절", OFFSET_OUTLINES, missing_walls=(OFFSET_GAP,),
                            checkpoints=OFFSET_GATES, grip_zones=(GripZone(5.9, 7.40, -0.46, 1.75, 0.68),)),
}
