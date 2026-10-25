
import json
import math
import urllib.request

import rclpy
from robot_controller import RobotController

OLLAMA_URL = "http://127.0.0.1:11434/api/generate"
MODEL = "qwen3:8b"


def ask_ai(instruction):
    prompt = f"""
あなたは移動ロボットの動作計画AIです。
日本語の指示をJSONに変換してください。

使用できる動作:
- move: 前進。distanceはメートル単位
- turn: 旋回。angleは度単位
  左回転は正、右回転は負

ルール:
- JSON以外は出力しない
- actions配列に動作を順番に入れる
- 最大5動作
- 前進距離は0.05〜1.0m
- 旋回角度は-180〜180度
- 対応できない指示はactionsを空配列にする
- 距離や角度が不明なら推測しない

例:
入力: 50cm前進して右に90度曲がって
出力:
{{"actions":[
  {{"type":"move","distance":0.5}},
  {{"type":"turn","angle":-90}}
]}}

入力:
{instruction}
"""

    data = {
        "model": MODEL,
        "prompt": prompt,
        "stream": False,
        "format": "json",
        "think": False,
        "options": {"temperature": 0}
    }

    request = urllib.request.Request(
        OLLAMA_URL,
        data=json.dumps(data).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST"
    )

    with urllib.request.urlopen(request, timeout=180) as response:
        result = json.load(response)

    return json.loads(result["response"])


def validate_plan(plan):
    if not isinstance(plan, dict):
        raise ValueError("JSON形式が正しくありません")

    actions = plan.get("actions")

    if not isinstance(actions, list) or not 1 <= len(actions) <= 5:
        raise ValueError("動作数が不正です")

    validated = []

    for action in actions:
        if not isinstance(action, dict):
            raise ValueError("動作形式が不正です")

        kind = action.get("type")

        if kind == "move":
            if set(action) != {"type", "distance"}:
                raise ValueError("前進命令の項目が不正です")

            value = action["distance"]

            if (
                type(value) not in (int, float)
                or not math.isfinite(value)
                or not 0.05 <= value <= 1.0
            ):
                raise ValueError("前進距離が範囲外です")

            validated.append(("move", float(value)))

        elif kind == "turn":
            if set(action) != {"type", "angle"}:
                raise ValueError("旋回命令の項目が不正です")

            value = action["angle"]

            if (
                type(value) not in (int, float)
                or not math.isfinite(value)
                or not 1 <= abs(value) <= 180
            ):
                raise ValueError("旋回角度が範囲外です")

            validated.append(("turn", float(value)))

        else:
            raise ValueError("対応していない動作です")

    return validated


def main():
    instruction = input("日本語で指示してください: ")

    print("Qwen3:8bが考えています...")

    try:
        plan = ask_ai(instruction)
        actions = validate_plan(plan)
    except Exception as e:
        print("AIの指示を実行できません:", e)
        return

    print("\nAIの判断結果:")
    print(json.dumps(plan, indent=2, ensure_ascii=False))

    answer = input("\nGazeboで実行しますか？ yes/no: ")

    if answer.strip().lower() != "yes":
        print("動作を実行せず終了しました")
        return

    rclpy.init()
    node = RobotController()

    try:
        for kind, value in actions:
            if kind == "move":
                print(f"前進：{value}m")
                node.move_forward(value)

            elif kind == "turn":
                print(f"旋回：{value}度")
                node.turn(value)

        print("すべての動作が正常に完了しました")

    except (RuntimeError, KeyboardInterrupt) as e:
        print(f"動作を中断しました：{e}")

    finally:
        for _ in range(10):
            node.send_velocity()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
