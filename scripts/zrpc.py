#!/usr/bin/env python
from __future__ import print_function
from collections import namedtuple
import json, urllib2, types, argparse, multiprocessing, sys, os, httplib, time, shlex, subprocess

config = None
parent = ""
if 'ZRPCPID' in os.environ:
	parent = os.environ['ZRPCPID']

def robust_exec(cmds):
	"""A function to execute a command locally. Because this function will run
	the command natively, the command will not be logged in the zrpc system.
	Consequently, the computing resource of the worker machine cannnot be holded.
	Therefore, this function can only be used to call some very lightweighted
	command such as a script coordinating complicated computation.

	Moreover, this function is used because in Linux, there is some unexpected behavior
	in using SIGINT to kill a subprocess with Python 2.x. More investigations are needed
	for Python 3.

	Args:
		cmds: A list of command-line arguments or a string of command line command.
			  e.g. ['del', 'file2'] or 'del file2'. The former form is recommanded

	Returns:
		0 for no error. Otherwise, non-zero value for an error code

	Raises:
		No exception
	"""
	import platform, os, time, inspect, subprocess, shlex
	src_file_name = os.path.basename(inspect.getfile(inspect.currentframe()))
	if platform.system() == 'Linux':
		if os.environ.get('LD_LIBRARY_PATH'):
			os.environ['LD_LIBRARY_PATH'] += ":" + os.path.dirname(os.path.abspath(os.sys.argv[0]))
		else:
			os.environ['LD_LIBRARY_PATH'] = os.path.dirname(os.path.abspath(os.sys.argv[0]))
#		print(os.path.dirname(os.sys.argv[0]))
		os.umask(002)

	if type(cmds) == str:
		print(cmds)
		p = subprocess.Popen(shlex.split(cmds.encode('string-escape')), env=os.environ)
		try:
			p.communicate()
			return p.returncode
		except:
			print('[{0}] exception in robust_exec()'.format(src_file_name))
			if platform.system() == 'Linux':
				p.send_signal(2)
			else:
				p.kill()
			while p.poll() == None:
				print('[{0}] waiting the child exit'.format(src_file_name))
				time.sleep(0.05)
				pass
			exit(1)
	else:
		cmds = [str(cmd) for cmd in cmds]
		print(' '.join(cmds))
		p = subprocess.Popen(cmds, env=os.environ)
		try:
			p.communicate()
			return p.returncode
		except:
			print('[{0}] exception in robust_exec()'.format(src_file_name))
			if platform.system() == 'Linux':
				p.send_signal(2)
			else:
				p.kill()
			while p.poll() == None:
				print('[{0}] waiting the child exit'.format(src_file_name))
				time.sleep(0.05)
				pass
			exit(1)


def read_config(filename):
	try:
		global config
		f = open(filename)
		configj = json.load(f)
		config.baseURL = "http://%s:%s/cmd/" % (configj["MasterIP"], configj["Port"])
		f.close()
	except Exception as e:
		print(e)
		print("%s does not exist" % filename)
	return config

def init_config():
	global config
	if isinstance(config, types.NoneType):
		config = namedtuple('Config', ['totalCPU', 'baseURL'])
		config.baseURL = ''
		config.totalCPU = 0
		read_config(os.path.join(os.path.dirname(sys.argv[0]), 'conf_release.json'))

init_config()

'''
health returns the health information of zrpc,
which is a dict with keys: NumMember, NumCPU, NumGPU.
NumMember is the number of living worker.
NumCPU is the current available CPU.
NumGPU is the current available GPU.
'''
def health():
	result = {
		"NumMember": 0,
		"NumCPU": 0,
		"NumGPU": 0
	}
	if config.baseURL == '':
		return result
	req = urllib2.Request(config.baseURL + "health")
	try:
		response = urllib2.urlopen(req, timeout=2)
		text = response.read()
		print('health ' + text)
		result = json.loads(text)
	except Exception as e:
		print(e)
		pass
	return result

def wait(reply, retry_timeout = 1, retry_times = 5):
	"""Wait for the mapped command to return. During the waiting, if there is
	network error or remote server restarts, the wait will be tried once again.

	Args:
		reply: The message to-be-send to master:port/wait to tell which job is waited.


	Returns:
		True if the commands success remotely
		False if the commands fail remotely or remote server downs

	Raises:
		httplib.BadStatusLine is raised, if the remote master is closed.
	"""
	try:
		req = urllib2.Request(config.baseURL+"wait")
		req.add_header('Content-Type', 'application/json')
		response = urllib2.urlopen(req, reply)
		text = response.read()
		print('wait for id: "{}" -> {}'.format(json.loads(reply)['Id'], text))
		if text == "Fail":
			return False
		return True
	except KeyboardInterrupt:
		req = urllib2.Request(config.baseURL+"kill")
		req.add_header('Content-Type', 'application/json')
		response = urllib2.urlopen(req, reply, timeout=20)
		print(response.read())
		exit(0)
	except (httplib.BadStatusLine, urllib2.URLError) as e:
		if (retry_times> 0):
			# retry to wait again after a timeout
			print('Remote server or network problem. Try to wait again after {} seconds'.format(retry_timeout))
			time.sleep(retry_timeout)
			return wait(reply, retry_timeout*2, retry_times - 1)
		else:
			print('Cannot connect to the remote any more. There must be some network error')
			exit(0)

def get_gpu_num():
	return subprocess.check_output("nvidia-smi --query-gpu=index --format=csv,noheader".split()).count('\n')

def get_cpu_num():
	return multiprocessing.cpu_count()

def split_option_file(argument_file_path, num):
	if os.path.exists(argument_file_path):
		if num == 1: return [argument_file_path]
		non_empty_lines = []
		with open(argument_file_path, 'rb') as argument_list_stream:
			for line in argument_list_stream.readlines():
				line = line.strip()
				if (len(line) > 0):
					non_empty_lines.append(line)
		if len(non_empty_lines) > 0:
			# split
			total_files = []
			total_lines = len(non_empty_lines)
			number_per_file = total_lines // num
			if number_per_file*num < total_lines: number_per_file += 1
			for i in range(0, num):
				start = i*number_per_file
				end = (i+1)*number_per_file
				if end >= total_lines: end = total_lines
				if end > start:
					split_path = argument_file_path + '.{}'.format(i)
					with open(split_path, 'wb') as split_stream:
						split_stream.write('\n'.join(non_empty_lines[start:end]))
					total_files.append(split_path)
			return total_files
		else:
			return []
	else:
		print('argument list file {} does not exist'.format(argument_file_path))
		return []

def exec_batch_in_file(cmds, input_index, threads_num, gpu, job_num):
	"""
	Returns:
		job_id, ok
		job_id, True for successful remote map
		job_id, False for unsuccessful remote map
		"", False for cannot connect to the remote server
		None, False for no remote config
	"""
	if config.baseURL == '':
		return None, False

	if job_num <= 0: job_num = health()['NumMember']
	if job_num <= 0: return "", False
	split_paths = split_option_file(cmds[input_index], job_num)
	cmdss = []
	for split_path in split_paths:
		new_cmd = cmds[0:input_index]
		new_cmd.append(split_path)
		new_cmd += cmds[input_index+1:]
		cmdss.append(new_cmd)
	return exec_batch(cmdss, threads_num, gpu, "", [])

# This "mulitple" type will be deprecated
def exec_multiple(cmds, input_index, threads_num, gpu, job_num):
	if config.baseURL == '':
		return None, False
	job = {
		"Type": "multiple",
		"Parent": parent,
		"Threads": threads_num,
		"GPU": gpu,
		"User": os.getenv("USER", "Anonymous"),
		"Input": input_index,
		"Args": cmds,
		"JobNumber": job_num
	}
	postdata = {
		"Priority": "Pro",
		"Type": "multiple",
		"Job": job
	}
	req = urllib2.Request(config.baseURL + "create")
	req.add_header('Content-Type', 'application/json')
	print(postdata)
	try:
		response = urllib2.urlopen(req, json.dumps(postdata), timeout=10).read()
	except Exception as e:
		print(e)
		return "", False
	job_id = json.loads(response)['Id']
	return job_id, wait(response)

def exec_batch(cmds, threads_num, gpu, depend_job, depend_tasks):
	"""
	Returns:
		job_id, ok
		job_id, True for successful remote map
		job_id, False for unsuccessful remote map
		"", False for cannot connect to the remote server
		None, False for no remote config
		None, True for empty maps
	"""
	if config.baseURL == '':
		return None, False
	if len(cmds) == 0: return None, True
	job = {
		"Type": "batch",
		"Parent": parent,
		"Threads": threads_num,
		"GPU": gpu,
		"User": os.getenv("USER", "Anonymous"),
		"DependJob": depend_job,
		"DependTasks": depend_tasks,
		"BatchArgs": cmds
	}
	postdata = {
		"Priority": "Pro",
		"Type": "batch",
		"Job": job
	}
	req = urllib2.Request(config.baseURL + "create")
	req.add_header('Content-Type', 'application/json')
	print(postdata)
	try:
		response = urllib2.urlopen(req, json.dumps(postdata), timeout=10).read()
	except Exception as e:
		print(e)
		return "", False
	print(response)
	job_id = json.loads(response)['Id']
	return job_id, wait(response)

def exec_trunk(cmds, threads_num, gpu, depend_job, depend_tasks, trunk):
	if config.baseURL == '':
		return None, False
	job = {
		"Type": "trunk",
		"Parent": parent,
		"Threads": threads_num,
		"GPU": gpu,
		"Trunk": trunk,
		"User": os.getenv("USER", "Anonymous"),
		"DependJob": depend_job,
		"DependTasks": depend_tasks,
		"BatchArgs": cmds
	}
	postdata = {
		"Priority": "Pro",
		"Type": "trunk",
		"Job": job
	}
	req = urllib2.Request(config.baseURL + "create")
	req.add_header('Content-Type', 'application/json')
	print(postdata)
	try:
		response = urllib2.urlopen(req, json.dumps(postdata), timeout=10).read()
	except Exception as e:
		print(e)
		return "", False
	job_id = json.loads(response)['Id']
	return job_id, wait(response)


def exec_single(cmds, threads_num, gpu):
	if config.baseURL == '':
		return None, False
	job = {
		"Type": "single",
		"Parent": parent,
		"Threads": threads_num,
		"GPU": gpu,
		"User": os.getenv("USER", "Anonymous"),
		"Args": cmds
	}
	postdata = {
		"Priority": "Pro",
		"Type": "single",
		"Job": job
	}
	req = urllib2.Request(config.baseURL + "create")
	req.add_header('Content-Type', 'application/json')
	print(postdata)
	try:
		response = urllib2.urlopen(req, json.dumps(postdata), timeout=10).read()
	except Exception as e:
		print(e)
		return False
	job_id = json.loads(response)['Id']
	return job_id, wait(response)

def map_local(cmds, thread_per_cmd = 1, cpu_num = 0, gpu = False):
	"""A function to map a list of command-line commands
	to the local machine according to cpu_num

	Args:
		cmds: A list of command-line commands. Each command is also in the form
			  of a list of arguments. e.g. [['del', 'file1'],['del', 'file2']]
			  The arguments can be of non-string simple type. This function will
			  automatically handle the type conversion.
		thread_per_cmd: Specify the number of threads each command occupies. A value
			  smaller than 1 will indicate each command will take over all the
			  cores of the local machine.
		cpu_num: Indicate the number of CPUs of the local machine. If this argument is
				 set to 0, then for local execution, the number of cores will be
				 set as cpu_num.
		gpu: Useless flag for local map

	Returns:
		True if all cmds success. Otherwise, False

	Raises:
		No exception
	"""
	if len(cmds) == 0: return

	if thread_per_cmd <= 0:
		thread_per_cmd = multiprocessing.cpu_count()

	if cpu_num == 0:
		cpu_num = multiprocessing.cpu_count()

	process_num = cpu_num / thread_per_cmd

	process_num = min(process_num, len(cmds))
	process_num = max(process_num, 1)

	print('Process #: {0}'.format(process_num))

	if (process_num > 1):
		threadPool = multiprocessing.Pool(processes = process_num)
		p = threadPool.map_async(robust_exec, iterable = cmds, chunksize = 1)
		try:
			results = p.get(0xFFFFFFFF)
			successful = True
			for result in results:
				successful = successful and result == 0
			threadPool.terminate()
			threadPool.join()
			return successful
		except KeyboardInterrupt:
			print('parent received control-c')
			threadPool.terminate()
			threadPool.join()
			exit(1)
	else:
		successful = True
		for cmd in cmds:
			successful = successful and robust_exec(cmd) == 0
		return successful

def map(cmds, thread_per_cmd = 1, cpu_num = 0, gpu = False, job_num = 0,
		depend_job = '', depend_tasks = [], trunk = 0):
	"""A unified function to map a list of command-line commands
	to either remote servers or a local machine.

	This function will first try to map the commands to remote
	servers specifiy in the config.json. If remote mapping does not work,
	it will fallback to the local machine. The cpu_num is only useful
	for local execution

	Args:
		cmds: A list of command-line commands. Each command is also in the form
			of a list of arguments. e.g. [['del', 'file1'],['del', 'file2']]
			The arguments can be of non-string simple type. This function will
			automatically handle the type conversion.
		thread_per_cmd: Specify the number of threads each command occupies. A value
			of -1 will indicate each command will take over all the cores of a server.
		cpu_num: Indicate the number of CPUs. This value is valid only for
			the fall-back execution on a local machine. If this argument is
			set to 0, then for local execution, the number of cores will be
			set as cpu_num.
		gpu: bool variable to indicate if the cmds require gpu
		job_num: Specify the number of split jobs for multiple type cmd.
			It is set 0 by default, which means the remote server will set it.
		depend_job: the new job's input depends on the dpend_job's output.
		depend_tasks: how the new job depends on the depend_job's tasks.
	Returns:
		job_id, ok
		job_id, True for successful remote map
		job_id, False for unsuccessful remote map
		None, True for good local map
		None, False for bad local map
	Raises:
		No exception
	"""
	job_id = None
	ok = True
	if not isinstance(cmds, types.ListType):
		print('Error, please pass a list of commands or a list of arguments')
		print(cmds)
		return job_id, False

	if len(cmds) == 0: return job_id, True

	print('Mapping the jobs to remote server')
	if isinstance(cmds[0], types.ListType):
		cmds = [[str(c) for c in cmd] for cmd in cmds]
		if trunk > 0:
			job_id, ok = exec_trunk(cmds, thread_per_cmd, gpu, depend_job, depend_tasks, trunk)
		else:
			job_id, ok = exec_batch(cmds, thread_per_cmd, gpu, depend_job, depend_tasks)
		if not ok and job_id == None:
			# fallback to local multiprocessing
			# only if no master is configured.
			print('Remote mapping failed')
			print('Mapping the jobs to local server')
			res = map_local(cmds, thread_per_cmd, cpu_num, gpu)
			return job_id, res
		elif not ok and job_id != None:
			print('Remote map failed!!! Master is set, but connection fails')
			return job_id, False
	else:
		cmds = [str(cmd) for cmd in cmds]
		input_index, cmds = int(cmds[0]), cmds[1:]
		# job_id, ok = exec_multiple(cmds, input_index, thread_per_cmd, gpu, job_num)
		job_id, ok = exec_batch_in_file(cmds, input_index, thread_per_cmd, gpu, job_num)
		if not ok and job_id == None:
			# fallback to local multiprocessing
			# only if not master is configured.
			print('Remote mapping failed')
			print('Mapping the jobs to local server')
			return job_id, robust_exec(cmds) == 0
		elif not ok and job_id != None:
			print('Remote map failed!!! Master is set, but connection fails')
			return job_id, False
	global parent
	if parent == "":
		parent = job_id
	return job_id, ok

def kill(configfile, cmdId):
	global config
	config = read_config(configfile)
	req = urllib2.Request(config.baseURL+"kill")
	req.add_header('Content-Type', 'application/json')
	response = urllib2.urlopen(req, json.dumps({"Id": cmdId}), timeout=3)
	print(response.read())


def runlist(configfile, threads_per_cmd, gpu, cmdfile):
	global config
	config = read_config(configfile)
	cmds = []
	with open(cmdfile, 'rb') as cmd_stream:
		for line in cmd_stream.readlines():
			line = line.strip()
			if (len(line) > 0):
				cmd = shlex.split(line)
				cmds.append(cmd)
	print(cmds)
	print(map(cmds, threads_per_cmd, 0, gpu))

def runarg(configfile, threads_per_cmd, gpu, index, cmd_string):
	global config
	config = read_config(configfile)
	cmds = shlex.split(cmd_string)
	print(cmds)
	print(map([index]+cmds, threads_per_cmd, 0, gpu))


def modify_cluster(configfile):
	cluster = raw_input("Enter new cluster name: ")
	master = raw_input("Enter master ip: ")
	worker = raw_input("Enter workers ip(seperated by space): ")
	workers = worker.split()
	print("Cluster name is", cluster)
	print("Master is", master)
	print("Workers are", workers)
	sure = raw_input("Modify cluster? (y/n): ")
	if sure != 'y':
		return
	req = urllib2.Request("http://%s:4204/admin/becomeMaster" % master)
	req.add_header('Content-Type', 'application/json')
	response = urllib2.urlopen(req, json.dumps({"Cluster": cluster}))
	print("modify master", master, response.read())

	for worker in workers:
		req = urllib2.Request("http://%s:4204/admin/becomeWorker" % worker)
		req.add_header('Content-Type', 'application/json')
		response = urllib2.urlopen(req, json.dumps({"Master": master}))
		print("modify worker", worker, response.read())


if __name__ == "__main__":
	parser = argparse.ArgumentParser(prog='zrpc', description = 'zrpc command line tool')
	parser.add_argument('-c', '--config', help = "The path of a configuration file of a cluster", default = "conf_release.json")
	subparsers = parser.add_subparsers()

	runlistparser = subparsers.add_parser('runlist', help = 'Run a list of commands in distributed clusters')
	runlistparser.add_argument('-t', '--threads_per_cmd', type=int, help = 'The number of threads per cmd', default = "1")
	runlistparser.add_argument('--gpu',dest='gpu',action='store_true', help='Use GPU')
	runlistparser.set_defaults(gpu=False)
	runlistparser.add_argument('cmdlist', help = 'The path of a list of commands')
	runlistparser.set_defaults(func=lambda args: runlist(args.config, args.threads_per_cmd, args.gpu, args.cmdlist))

	runargparser = subparsers.add_parser('runarg', help = 'Run a command whose option will be mapped in distributed clusters')
	runargparser.add_argument('-t', '--threads_per_cmd', type=int, help = 'The number of threads per cmd', default = "1")
	runargparser.add_argument('--gpu',dest='gpu',action='store_true', help='Use GPU')
	runargparser.set_defaults(gpu=False)
	runargparser.add_argument('index', type=int, help = 'The index of arguments to be splited')
	runargparser.add_argument('args_string', help = 'The path of a list of commands')
	runargparser.set_defaults(func=lambda args: runarg(args.config, args.threads_per_cmd, args.gpu, args.index, args.args_string))

	killparser = subparsers.add_parser('kill', help = 'Kill a job with given ID')
	killparser.add_argument('ID', help = 'The ID of job to be killed')
	killparser.set_defaults(func=lambda args: kill(args.config, args.ID))

	clusterparser = subparsers.add_parser('cluster', help = 'modify cluster mater and worker')
	clusterparser.set_defaults(func=lambda args: modify_cluster(args.config))

	args = parser.parse_args()
	args.func(args)
