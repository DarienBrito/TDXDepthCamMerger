"""
Save DemoTDXDepthCamMerger.toe from the OPEN master, then bring the master back.

The demo IS the master minus TDMCP: the same component, the same Inputs_1 /
Inputs_2 synthetic cameras, the same nodes in the same places. Only what is true
of this machine is cleared first: the Python exe, the worker probe and the last
calibration readouts. Nothing is added or moved. What a visitor reads is the
annotateDemo box beside the component, which is part of the master.

Run from TD, with the master open and the rig built:

    exec(open(r'<this file>', encoding='utf-8').read())

TDMCP lives outside the project, on this machine only, so a user opening the
demo would get a broken reference. Destroying it also cuts the connection this
call came in on, so the work waits three frames and writes each step to
Log/demo_save.log, which is the only record left once the connection is gone.

Nothing done here reaches the master: it is reloaded from disk at the end.
Expect TouchDesigner to be gone when it finishes. Relaunch it with the master.
"""

import os

REPO = globals().get('REPO') or project.folder.replace(os.sep, '/')
DEMO = REPO + '/DemoTDXDepthCamMerger.toe'
MASTER = REPO + '/TDXDepthCamMerger.0.2.toe'
LOG = REPO + '/Log/demo_save.log'
HOST = op('/ProjectName')
NAME = 'TDXDepthCamMerger'

if not os.path.isdir(REPO + '/Log'):
	os.makedirs(REPO + '/Log')

comp = HOST.op(NAME)
if comp is None:
	raise RuntimeError('no {} in the open project'.format(NAME))

# _______________________________________________ 1. drop what is local to here

comp.par.Pythonexe = ''
# The default ships too, and since td_build.py started setting defaults it holds
# this machine's conda path. Nothing restores it here because the master is
# reloaded from disk at the end.
comp.par.Pythonexe.default = ''
for name in ('Open3dstatus', 'Open3dversion', 'Pythonversion',
		'Laststatus', 'Lastfitness', 'Lastrmse', 'Lastoverlap'):
	comp.par[name] = ''
comp.par.Lastcorrespondences = 0

print('cleaned: Python exe, worker probe, last readouts')

# _______________________________________________ 2. the save, three frames on
#
# chr(10) instead of a newline escape: this code is a string inside a string,
# and a real newline here would break the generated call apart.

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
