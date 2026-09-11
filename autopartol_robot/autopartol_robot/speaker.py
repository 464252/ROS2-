#!/usr/bin/env python3
import asyncio
import subprocess
import threading

import edge_tts
import rclpy
from rclpy.node import Node
from autopartol_interfaces.srv import SpeachText

class Speaker(Node):
    def __init__(self, name):
        super().__init__(name)
        self.speech_service_ = self.create_service(
            SpeachText, 'speech_text', self.speech_text_callback
        )
        self.get_logger().info('语音服务已启动，使用 edge-tts 引擎')
        # ==========================================
        # 就是这行！想换声音改这里
        self.voice_ = 'zh-CN-XiaoxiaoNeural'
        # 其他可选：zh-CN-YunxiNeural（男声）
        #           zh-CN-XiaohanNeural（知性女声）
        # ==========================================

        # 独立事件循环线程，避免回调中 asyncio.run() 冲突
        self.loop_ = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(
            target=self.loop_.run_forever, daemon=True
        )
        self._loop_thread.start()

    def speech_text_callback(self, request, response):
        self.get_logger().info(f'正在准备朗读 {request.text}')
        try:
            communicate = edge_tts.Communicate(request.text, self.voice_)
            
            # 用独立线程的事件循环跑异步任务
            future = asyncio.run_coroutine_threadsafe(
                communicate.save('/tmp/tts_output.mp3'),
                self.loop_
            )
            future.result(timeout=30)
            
            subprocess.run(
                ['mpv', '/tmp/tts_output.mp3'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            response.result = True
        except Exception as e:
            self.get_logger().error(f'语音合成失败: {e}')
            response.result = False
        return response

def main():
    rclpy.init()
    node = Speaker('speaker')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()