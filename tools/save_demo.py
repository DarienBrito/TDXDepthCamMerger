"""
Save DemoTDXDepthCamMerger.toe, then bring the master back.

Run from TD once tools/td_build_demo.py has built the demo in the open project:

    exec(open(r'<this file>', encoding='utf-8').read())

The demo must not ship TDMCP: it is an external tox on this machine only, so a
user opening the demo would get a broken reference. Destroying it kills the MCP
connection this call arrived on, so the work is deferred by three frames and
every step is logged with a flush. MCP returns nothing once the process is gone,
which makes Log/demo_save.log the only evidence of what happened.

Expect TouchDesigner to be gone when it finishes. Relaunch it with the master:
the port came back in 4 s when this was measured on 2026-09-01.
"""

import os

REPO = globals().get('REPO') or project.folder.replace(os.sep, '/')
DEMO = REPO + '/DemoTDXDepthCamMerger.toe'
MASTER = REPO + '/TDXDepthCamMerger.0.2.toe'
LOG = REPO + '/Log/demo_save.log'

if not os.path.isdir(REPO + '/Log'):
	os.makedirs(REPO + '/Log')

# chr(10) rather than an escape: this source is a string inside a string, and a
# real newline here would land in the middle of the generated call.
CODE = '''
import os
import traceback

log = open({log!r}, 'w')


def w(msg):
	log.write(msg + chr(10))
	log.flush()


w('--- start')
try:
	tdmcp = op('/ProjectName/TDMCP')
	w('tdmcp: ' + str(tdmcp))
	if tdmcp:
		tdmcp.destroy()
	w('tdmcp destroyed')
	w('project.save -> ' + str(project.save({demo!r})))
	w('demo bytes: ' + str(os.path.getsize({demo!r})))
	w('loading master')
	project.load({master!r})
	w('--- master reloaded')
except Exception:
	w(traceback.format_exc())
log.close()
'''.format(log=LOG, demo=DEMO, master=MASTER)

run(CODE, delayFrames=3)
print('save scheduled in 3 frames. Watch {}'.format(LOG))
print('TouchDesigner is expected to exit; relaunch it with {}'.format(MASTER))
