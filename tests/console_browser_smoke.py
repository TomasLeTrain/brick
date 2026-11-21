"""Optional end-to-end console smoke test. Requires Playwright + Chromium.
Runs a local server and fake USB/gamepad; does not access real hardware.
"""
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from threading import Thread
import json
import math
import os
import shutil
from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parents[1]
class QuietHandler(SimpleHTTPRequestHandler):
    def log_message(self, *args):
        pass

MOCK = r"""
window.sent = [];
window.pad = {index:0,id:'Joy-Con test',connected:true,axes:[0,0],buttons:Array.from({length:16},()=>({pressed:false,value:0}))};
Object.defineProperty(navigator,'getGamepads',{value:()=>[window.pad]});
let incoming;
const fake = {
 async open() { this.readable=new ReadableStream({start(c){incoming=c;}});this.writable=new WritableStream({write(chunk){window.sent.push(new TextDecoder().decode(chunk));}}); },
 async close(){},
};
window.receiveText=text=>incoming.enqueue(new TextEncoder().encode(text));
window.endSerial=()=>incoming.close();
Object.defineProperty(navigator,'serial',{value:{requestPort:async()=>fake}});
"""

def main():
    server = ThreadingHTTPServer(('127.0.0.1', 0), partial(QuietHandler, directory=ROOT/'tools/console'))
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        with sync_playwright() as p:
            browser = p.chromium.launch(headless=True, executable_path=os.environ.get('BRICK_CHROMIUM') or shutil.which('chromium'), args=['--no-sandbox'])
            page = browser.new_page(viewport={'width':1440,'height':1100}, device_scale_factor=1)
            errors = []
            page.on('pageerror', lambda error: errors.append(str(error)))
            page.add_init_script(MOCK)
            page.goto(f'http://127.0.0.1:{server.server_port}/')
            page.set_default_timeout(8000)
            page.bring_to_front()
            page.wait_for_function("document.querySelector('#gamepad').textContent.includes('Joy-Con test')")
            page.locator('summary').click()
            page.locator('#captureButton').select_option('3')
            page.locator('#exportButton').select_option('4')
            page.locator('summary').click()
            page.locator('#connect').click()
            page.wait_for_function("document.querySelector('#serialState').textContent === 'Sender USB connected'")
            assert not page.locator('#arm').is_disabled()
            page.evaluate('window.pad.axes[0]=.7')
            page.locator('#arm').click()
            page.wait_for_function("window.sent.some(s=>/^@C,1,\\d+,\\d+,[1-9]\\d*,-1,1\\n$/.test(s))")
            page.evaluate('window.pad.buttons[1].pressed=true')
            page.wait_for_function("document.querySelector('#inverted').getAttribute('aria-pressed') === 'true'")
            page.wait_for_timeout(250)
            assert page.locator('#inverted').get_attribute('aria-pressed') == 'true'
            assert page.evaluate("window.sent.filter(s=>s.startsWith('@C,') && s.trim().split(',')[5] === '1').length") == 1
            page.evaluate('window.pad.buttons[1].pressed=false;window.pad.buttons[0].pressed=true')
            page.wait_for_function("document.querySelector('#upright').getAttribute('aria-pressed') === 'true'")
            page.wait_for_function("window.sent.filter(s=>s.startsWith('@C,') && s.trim().split(',')[5] === '0').length === 1")
            page.evaluate('window.pad.buttons[0].pressed=false')
            page.wait_for_timeout(50)
            page.evaluate('window.pad.buttons[0].pressed=true')
            page.wait_for_function("window.sent.filter(s=>s.startsWith('@C,') && s.trim().split(',')[5] === '0').length === 2")
            page.evaluate('window.pad.buttons[0].pressed=false')
            page.evaluate('window.pad.buttons[1].pressed=false;window.pad.buttons[3].pressed=true')
            page.wait_for_function("window.sent.includes('s\\n')")
            page.evaluate('window.pad.buttons[3].pressed=false')
            telemetry = dict.fromkeys(['v','link','have_telemetry','rx_age_ms','rpm','target_rpm','throttle','rssi_hz',
                                       'correlation','heading_rad','heading_error_rad','rpm_age_ms','rssi_dbm','control_age_ms',
                                       'control_sequence','connection_state','rpm_status','heading_status','inverted',
                                       'companion_active','controller_connected','spin_enabled','rx_dropped','rssi_dropped',
                                       'robot_tx_failures','tx_attempts','tx_completed','tx_failures','tx_max_us','pc_active',
                                       'sender_inverted','channel'], 0)
            telemetry.update(v=1,link=1,have_telemetry=1,rpm=2175,target_rpm=2200,throttle=.24,rssi_hz=875,
                             correlation=.93,rssi_dbm=-58,rpm_age_ms=90,control_age_ms=8,connection_state=4,
                             rpm_status=6,companion_active=1,controller_connected=1,tx_max_us=2100,channel=6,
                             sender_inverted=1)
            page.evaluate('text=>window.receiveText(text)', '@T '+json.dumps(telemetry)+'\n')
            page.wait_for_function("document.querySelector('#rpm').textContent.includes('2,175')")
            page.screenshot(path='/tmp/brick-console-desktop.png', full_page=True)
            # Physical disconnect releases ownership and requires explicit re-enable.
            page.evaluate('window.pad.connected=false')
            page.wait_for_function("document.querySelector('#arm').textContent.includes('Enable companion')")
            page.wait_for_function("window.sent.at(-1).trim().endsWith(',0')")
            page.evaluate('window.pad.connected=true;window.pad.buttons[3].pressed=false')
            page.wait_for_timeout(100)
            assert 'Enable companion' in page.locator('#arm').inner_text()
            # CSV export can be copied/downloaded only once its end marker arrives.
            fields='kind,timestamp_us,received_us,rssi_dbm,target_rpm,rssi_rpm,throttle,analysis_timestamp_us,comparison_samples,min_lag,max_lag,rpm_status'
            lines=['# dump_begin generation=1 count=601',fields]
            for i in range(600):
                lines.append(f'1,{1000000+i*1000},{1000000+i*1000},{-60+10*math.sin(i*math.tau/40)},0,0,0,0,0,0,0,0')
            lines.append('2,1599000,1599000,0,1500,1500,.2,1599000,250,9,155,6')
            page.evaluate('text=>window.receiveText(text)', '\n'.join(lines)+'\n')
            assert page.locator('#saveLog').is_disabled()
            page.evaluate('text=>window.receiveText(text)', '# dump_end\n')
            page.wait_for_function("!document.querySelector('#saveLog').disabled")
            page.locator('[data-view=logs]').click()
            assert '601 records' in page.locator('#logState').inner_text()
            page.locator('#cursor').focus()
            page.locator('#cursor').press('Home')
            page.locator('#cursor').press('End')
            with page.expect_download() as info:
                page.locator('#saveLog').click()
            path = info.value.path()
            assert Path(path).read_text().endswith('# dump_end\n')
            page.screenshot(path='/tmp/brick-console-logs.png', full_page=True)
            page.locator('#file').set_input_files(ROOT/'logs/log18.csv')
            page.wait_for_function("document.querySelector('#logState').textContent.includes('2048 records')")
            # Window focus loss releases a currently enabled controller.
            page.locator('#arm').click()
            page.evaluate("window.dispatchEvent(new Event('blur'))")
            page.wait_for_function("document.querySelector('#arm').textContent.includes('Enable companion')")
            page.set_viewport_size({'width':390,'height':844})
            page.screenshot(path='/tmp/brick-console-mobile.png', full_page=True)
            assert page.evaluate('document.documentElement.scrollWidth <= innerWidth')
            page.evaluate('window.endSerial()')
            page.wait_for_function("document.querySelector('#serialState').textContent === 'Serial disconnected'")
            page.wait_for_function("!document.querySelector('#connect').disabled")
            page.locator('#connect').click()
            page.wait_for_function("document.querySelector('#serialState').textContent === 'Sender USB connected'")
            assert 'Enable companion' in page.locator('#arm').inner_text()
            page.evaluate('window.endSerial()')
            page.wait_for_function("!document.querySelector('#connect').disabled")
            assert not errors, errors
            browser.close()
            print('Browser smoke passed: serial, gamepad, independent orientation actions, capture, telemetry, CSV, disconnect, responsive layout')
    finally:
        server.shutdown()
        server.server_close()

if __name__ == '__main__':
    main()
