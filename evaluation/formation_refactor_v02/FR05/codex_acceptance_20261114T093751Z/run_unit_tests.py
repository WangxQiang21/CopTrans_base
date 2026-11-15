from pathlib import Path
import sys,os,subprocess,json,time
OUT=Path(__file__).parent;E=os.environ.copy();E.update(PYTHONDONTWRITEBYTECODE="1",OPENBLAS_NUM_THREADS="1")
for k in ("FR01_RESULTS_PATH","FR02_TIMING_RESULTS_PATH","FR02_TRAJECTORY_RESULTS_PATH","FR02_QUALITY_RESULTS_PATH","FR03_RESULTS_PATH","FR04_RESULTS_PATH"):E[k]=str(OUT/(k.lower()+".json"))
for k in ("FR02_ARTIFACT_DIR","FR03_ARTIFACT_DIR","FR04_ARTIFACT_DIR"):E[k]=str(OUT/k.lower());Path(E[k]).mkdir(exist_ok=True)
groups={"unit_61":["test_formation_demo_assets","test_uniform_trajectory","test_uniform_timing","test_uniform_motion","test_uniform_playback_plan"],"unit_84":["test_uniform_pipeline","test_uniform_search","test_uniform_quality","test_formation_playback","test_formation_hold","test_transport_contract"],"unit_node":["test_formation_optimizer_node"]};rows=[]
for name,mods in groups.items():
 code="import unittest,sys;mods="+repr(["tests."+s for s in mods])+";s=unittest.TestSuite();loader=unittest.defaultTestLoader;[s.addTests(loader.loadTestsFromName(m)) for m in mods];flat=lambda s:[t for x in s for t in (flat(x) if isinstance(x,unittest.TestSuite) else [x])];s=unittest.TestSuite(x for x in flat(s) if 'test_S09_' not in x.id());r=unittest.TextTestRunner(verbosity=2).run(s);sys.exit(not r.wasSuccessful())"
 cmd=[sys.executable,"-c",code];t=time.perf_counter()
 with (OUT/(name+".log")).open("w") as f:r=subprocess.run(cmd,env=E,stdout=f,stderr=subprocess.STDOUT)
 rows.append(dict(group=name,command=cmd,exit_code=r.returncode,wall_s=time.perf_counter()-t));(OUT/"test_commands.json").write_text(json.dumps(rows,indent=2));print(rows[-1],flush=True)
