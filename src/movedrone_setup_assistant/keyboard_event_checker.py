import sys
import select
import termios
import tty
import rospy
from std_msgs.msg import Empty


class KeyboardEventChecker:
    """ Ctrl+Cを検出してアプリ終了を要求するためのノード． """

    GETKEY_TIMEOUT = 1.

    def __init__(self):
        self._settings = termios.tcgetattr(sys.stdin)

        self._quit_pub = rospy.Publisher("/quit_movedrone", Empty, queue_size=1)

    def run(self):
        while not rospy.is_shutdown():
            key = self._get_key()

            # Ctrl+Cを検出したらアプリ終了を要求する
            if key == "\x03":
                self._quit_pub.publish(Empty())
                break

    def _get_key(self) -> str:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], self.GETKEY_TIMEOUT)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._settings)
        return key
