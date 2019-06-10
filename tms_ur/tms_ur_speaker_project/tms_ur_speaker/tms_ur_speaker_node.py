import rclpy
# from tms_ur_speaker.srv import SpeakerSrv
from std_msgs.msg import String
import subprocess
import os

base = os.path.dirname(os.path.abspath(__file__))

def jtalk(t):
    open_jtalk=['open_jtalk']
    mech=['-x','/var/lib/mecab/dic/open-jtalk/naist-jdic']
    htsvoice=['-m','/usr/share/hts-voice/mei/mei_normal.htsvoice']
    speed=['-r','1.0']
    quality=['-a','0.57']
    outwav=['-ow','open_jtalk.wav']
    cmd=open_jtalk+mech+htsvoice+speed+quality+outwav
    c = subprocess.Popen(cmd,stdin=subprocess.PIPE)
    c.stdin.write(t.encode())
    c.stdin.close()
    c.wait()
    aplay = ['aplay','-q','open_jtalk.wav']
    wr = subprocess.Popen(aplay)

def speak(data):
    if data == '':
        return 0
    elif data[0]=='\\':
        aplay = ['aplay','-q',os.path.join(base, '../wav/'+data[1:]+'.wav')]
        wr = subprocess.Popen(aplay)
        soxi = ['soxi','-D',os.path.join(base, '../wav/'+data[1:]+'.wav')]
        ret = subprocess.check_output(soxi)
        print(ret)
        return ret
    else:
        talk = data.replace(',','')
        jtalk(talk)
        soxi = ['soxi','-D',os.path.join(base, '../wav/open_jtalk.wav')]
        ret = subprocess.check_output(soxi)
        print(ret)
        return ret

def subscription_callback(msg):
    global g_node
    g_node.get_logger().info(
        'Speaker: "%s"' % msg.data
    )
    jtalk(msg.data)


def main(args=None):
    global g_node
    rclpy.init(args=args)

    g_node = rclpy.create_node('tms_ur_speaker')
    print('tms_ur_speaker : '+base)

    subscription = g_node.create_subscription(String, 'speaker', subscription_callback, 10)
    subscription  # prevent unused variable warning

    while rclpy.ok():
        rclpy.spin_once(g_node)

    g_node.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()