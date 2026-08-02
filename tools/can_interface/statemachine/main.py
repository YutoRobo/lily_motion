# -*- coding: utf-8 -*-

import argparse
import os
import rospy
import can
from unified_state_machine import StateMachine

# sudo ip link set can0 up type can bitrate 500000
# sudo ip link set can0 up

def parse_args():
    parser = argparse.ArgumentParser(description='CAN StateMachine runner')
    parser.add_argument('--can-interface', default=os.environ.get('LILY_CAN_INTERFACE', 'socketcan'))
    parser.add_argument('--can-channel', default=os.environ.get('LILY_CAN_CHANNEL', 'can0'))
    parser.add_argument('--can-bitrate', type=int, default=int(os.environ.get('LILY_CAN_BITRATE', '500000')))
    return parser.parse_args()


def main():
    args = parse_args()
    # ROSノードの初期化
    rospy.init_node('robot_state_machine', anonymous=True)
    # CANバスの初期化
    try:
        bus = can.interface.Bus(interface=args.can_interface, channel=args.can_channel, bitrate=args.can_bitrate)
        rospy.loginfo("CAN bus ready: interface=%s channel=%s bitrate=%d", args.can_interface, args.can_channel, args.can_bitrate)
    except Exception as e:
        rospy.logfatal("CAN bus init failed: interface=%s channel=%s bitrate=%d error=%s", args.can_interface, args.can_channel, args.can_bitrate, e)
        return

    # /cmdForJetson一本化・Use=True軸のみ送信するStateMachine
    sm = StateMachine(bus)

    # CAN受信用のリスナーを設定
    # メッセージを受信するたびに sm.can_callback が呼ばれる
    can_listener = can.BufferedReader()
    can_listener.on_message_received = sm.can_callback
    notifier = can.Notifier(bus, [can_listener])

    rospy.loginfo("ステートマシンを起動しました。Ctrl+Cで停止します。")

    rate = rospy.Rate(30)
    try:
        while not rospy.is_shutdown():
            # ステートマシンの周期処理を実行
            sm.execute()
            rate.sleep()

    except KeyboardInterrupt:
        rospy.loginfo("シャットダウンします。")
    finally:
        notifier.stop()
        bus.shutdown()
        rospy.loginfo("CANバスをシャットダウンしました。")

if __name__ == '__main__':
    main()
