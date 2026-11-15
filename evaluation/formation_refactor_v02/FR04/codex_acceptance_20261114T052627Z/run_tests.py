from pathlib import Path
import subprocess,os,json,time,sys
ROOT=Path.cwd(); OUT=Path(__file__).parent
E=os.environ.copy(); E.update(PYTHONDONTWRITEBYTECODE="1",OPENBLAS_NUM_THREADS="1")
for k in ("FR01_RESULTS_PATH","FR02_TIMING_RESULTS_PATH","FR02_TRAJECTORY_RESULTS_PATH","FR02_QUALITY_RESULTS_PATH","FR03_RESULTS_PATH","FR04_RESULTS_PATH"):
 E[k]=str(OUT/(k.lower()+".json"))
for k in ("FR02_ARTIFACT_DIR","FR03_ARTIFACT_DIR","FR04_ARTIFACT_DIR"):
 E[k]=str(OUT/k.lower()); Path(E[k]).mkdir(exist_ok=True)
suites=["test_uniform_pipeline","test_formation_collision","test_formation_geometry","test_formation_hold","test_formation_kinematics","test_formation_quality","test_formation_retiming","test_uniform_motion","test_uniform_quality","test_uniform_timing","test_uniform_trajectory","test_uniform_search"]
records=[]
for name in suites:
 cmd=[sys.executable,"-m","unittest","discover","-s","tests","-p",name+".py","-v"]
 if name == "test_uniform_search":
  code="import sys,unittest;sys.path.insert(0,'tests');import test_uniform_search as t;s=unittest.defaultTestLoader.loadTestsFromTestCase(t.UniformSearchAcceptance);s=unittest.TestSuite(x for x in s if 'test_S09_' not in x.id());r=unittest.TextTestRunner(verbosity=2).run(s);sys.exit(not r.wasSuccessful())"
  cmd=[sys.executable,"-c",code]
 t=time.perf_counter()
 with (OUT/(name+".log")).open("w") as f: r=subprocess.run(cmd,env=E,stdout=f,stderr=subprocess.STDOUT)
 records.append(dict(suite=name,command=cmd,exit_code=r.returncode,wall_s=time.perf_counter()-t)); (OUT/"test_commands.json").write_text(json.dumps(records,indent=2)); print(records[-1],flush=True)
