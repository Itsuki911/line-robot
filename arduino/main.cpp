#include "mbed.h"

using namespace std::chrono;

// ラインセンサー（S0が右端、S4が左端）
AnalogIn S0(A0);
AnalogIn S1(A1);
AnalogIn S2(A2);
AnalogIn S3(A3);
AnalogIn S4(A4);

// モータードライバー（assets/example.cppと同じ配線）
PwmOut PWM1(D11); // 左モーターの速度
PwmOut PWM2(D9);  // 右モーターの速度
DigitalOut DIR1(D12);
DigitalOut DIR2(PA_11_ALT0);

// HC-SR04互換の超音波センサー（Ultrasonic-sensor.inoと同じピン）
DigitalOut trig(D7);
DigitalIn echo(D8);

// 実機は黒地に白線のため、0.65以上を白線として扱う
constexpr float WHITE_THRESHOLD = 0.65f; // 白線と判定する反射センサー値のしきい値

constexpr float BASE_SPEED = 0.15f;   // 白線追従時の基本速度
constexpr float TURN_GAIN = 0.075f;  // 白線の位置誤差に対する旋回量
constexpr float SEARCH_SPEED = 0.10f; // 白線を見失ったときの探索速度

// センサー取付位置と停止距離に合わせて実機で調整する値
constexpr float OBSTACLE_DISTANCE_CM = 18.0f; // 障害物と判断する距離（cm）
constexpr auto ULTRASONIC_TIMEOUT = 30ms; // エコー受信を待つ最長時間
constexpr auto DISTANCE_SAMPLE_INTERVAL = 80ms; // 超音波センサーの測距間隔
constexpr auto OBSTACLE_REARM_DELAY = 2500ms; // 回避後に再検出を有効にするまでの時間

// 障害物回避の動作状態
enum class AvoidState {
    FollowLine,
    Stop,
    Reverse,
    TurnRight,
    PassObstacle,
    TurnLeft,
    FindLine,
};

AvoidState avoidState = AvoidState::FollowLine;
Timer stateTimer;
Timer distanceTimer;
Timer obstacleRearmTimer;
Timer debugTimer;
float lastLineError = 0.0f;
float lastDistanceCm = -1.0f;

// PWMデューティ比を0.0〜1.0の範囲に収める
float clampDuty(float value) {
    if (value < 0.0f) {
        return 0.0f;
    }
    if (value > 1.0f) {
        return 1.0f;
    }
    return value;
}

// 左右モーターの速度を設定する
void setMotors(float left, float right) {
    PWM1.write(clampDuty(left));
    PWM2.write(clampDuty(right));
}

void setForward(bool forward) {
    // assets/example.cppに合わせ、LOWを前進方向として設定する
    DIR1 = forward ? 0 : 1;
    DIR2 = forward ? 0 : 1;
}

// 左右のモーターを停止する
void stopMotors() {
    setMotors(0.0f, 0.0f);
}

// 超音波を測距してcmで返す。エコーを受信できない場合は-1を返す
float readDistanceCm() {
    trig = 0;
    wait_us(2);
    trig = 1;
    wait_us(10);
    trig = 0;

    Timer timer;
    timer.start();
    while (echo == 0) {
        if (timer.elapsed_time() >= ULTRASONIC_TIMEOUT) {
            return -1.0f;
        }
    }

    timer.reset();
    while (echo == 1) {
        if (timer.elapsed_time() >= ULTRASONIC_TIMEOUT) {
            return -1.0f;
        }
    }

    // 音速を約0.0343 cm/usとして、往復時間から片道距離を求める
    return duration_cast<microseconds>(timer.elapsed_time()).count() * 0.01715f;
}

// 5個の反射センサーを使って白線を追従する
void followWhiteLine() {
    const float values[5] = {
        S0.read(), S1.read(), S2.read(), S3.read(), S4.read()
    };
    const float positions[5] = {2.0f, 1.0f, 0.0f, -1.0f, -2.0f};

    float weightedPosition = 0.0f;
    float whiteCount = 0.0f;
    for (int i = 0; i < 5; ++i) {
        if (values[i] >= WHITE_THRESHOLD) {
            weightedPosition += positions[i];
            whiteCount += 1.0f;
        }
    }

    if (whiteCount == 0.0f) {
        // 全センサーで見失った場合は、最後に見えた方向を探索する
        if (lastLineError >= 0.0f) {
            setMotors(SEARCH_SPEED, 0.0f); // 右側を探索
        } else {
            setMotors(0.0f, SEARCH_SPEED); // 左側を探索
        }
        return;
    }

    const float error = weightedPosition / whiteCount;
    lastLineError = error;

    // 正の誤差は白線が右側にあることを示すため、左車輪を速くして右へ曲がる
    // 交差点では白線を検出した全センサーの平均位置を利用する
    setMotors(BASE_SPEED + TURN_GAIN * error,
              BASE_SPEED - TURN_GAIN * error);
}

// 障害物回避の状態遷移を開始する
void startAvoidance() {
    avoidState = AvoidState::Stop;
    stateTimer.reset();
    stateTimer.start();
    obstacleRearmTimer.reset();
    obstacleRearmTimer.start();
}

// 現在の状態に応じて障害物回避動作を実行する
void runAvoidance() {
    // 実機で時間を調整する。右へ避け、障害物通過後に左へ戻る
    switch (avoidState) {
    case AvoidState::Stop:
        // 障害物を検出した直後に安全のため停止する
        stopMotors();
        if (stateTimer.elapsed_time() >= 200ms) {
            avoidState = AvoidState::Reverse;
            stateTimer.reset();
        }
        break;
    case AvoidState::Reverse:
        // 少し後退して、旋回のための空間を作る
        setForward(false);
        setMotors(0.10f, 0.10f);
        if (stateTimer.elapsed_time() >= 150ms) {
            setForward(true);
            avoidState = AvoidState::TurnRight;
            stateTimer.reset();
        }
        break;
    case AvoidState::TurnRight:
        // 右へ旋回して障害物の横へ移動する
        setMotors(0.17f, 0.03f);
        if (stateTimer.elapsed_time() >= 380ms) {
            avoidState = AvoidState::PassObstacle;
            stateTimer.reset();
        }
        break;
    case AvoidState::PassObstacle:
        // 障害物を通り過ぎるまで直進する
        setMotors(0.14f, 0.14f);
        if (stateTimer.elapsed_time() >= 750ms) {
            avoidState = AvoidState::TurnLeft;
            stateTimer.reset();
        }
        break;
    case AvoidState::TurnLeft:
        // 元のライン方向へ戻るため左へ旋回する
        setMotors(0.03f, 0.17f);
        if (stateTimer.elapsed_time() >= 380ms) {
            avoidState = AvoidState::FindLine;
            stateTimer.reset();
        }
        break;
    case AvoidState::FindLine:
        // 白線追従を再開し、ラインへ復帰する
        followWhiteLine();
        if (stateTimer.elapsed_time() >= 1200ms) {
            avoidState = AvoidState::FollowLine;
        }
        break;
    case AvoidState::FollowLine:
        // 通常走行中はこの関数ではモーター操作を行わない
        break;
    }
}

// ロボットの初期化とメイン制御ループ
int main() {
    // モーターPWMの周期を10ms（100Hz）に設定する
    PWM1.period(0.01f);
    PWM2.period(0.01f);
    // モーターを前進方向にし、超音波の送信端子をLOWで初期化する
    setForward(true);
    trig = 0;

    // 測距、障害物再検出、デバッグ表示に使用するタイマーを開始する
    distanceTimer.start();
    obstacleRearmTimer.start();
    debugTimer.start();

    while (true) {
        // 回避中でなければ、白線追従と障害物監視を実行する
        if (avoidState == AvoidState::FollowLine) {
            followWhiteLine();

            // 指定間隔ごとに前方の距離を測定する
            if (distanceTimer.elapsed_time() >= DISTANCE_SAMPLE_INTERVAL) {
                distanceTimer.reset();
                lastDistanceCm = readDistanceCm();
                // 再検出待ち時間後、しきい値より近い物体を障害物として扱う
                if (obstacleRearmTimer.elapsed_time() >= OBSTACLE_REARM_DELAY &&
                    lastDistanceCm > 0.0f &&
                    lastDistanceCm < OBSTACLE_DISTANCE_CM) {
                    startAvoidance();
                }
            }
        } else {
            // 回避中は状態に対応したモーター操作だけを実行する
            runAvoidance();
        }

        // 80msごとの測距結果ではなく、1秒ごとにデバッグ出力する
        if (debugTimer.elapsed_time() >= 1000ms) {
            debugTimer.reset();
            if (lastDistanceCm > 0.0f) {
                printf("distance: %.1f cm\r\n", lastDistanceCm);
            } else {
                printf("distance: no echo\r\n");
            }
        }

        ThisThread::sleep_for(10ms);
    }
}
