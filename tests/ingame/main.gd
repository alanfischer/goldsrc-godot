extends Node
## Runner for the in-engine suites. Discovers every `suites/suite_*.gd`, runs it, and exits
## non-zero if anything failed. See run.sh.
##
## Unlike goldnet's equivalent this is NOT a standalone project: it runs against the repo
## root so res://addons/goldsrc (the GDExtension under test) and res://maps resolve exactly
## as they do in the real project. That is the whole point — these suites exercise the
## Godot-facing node classes, which cannot be reached from the standalone C++ tests.
##
##   godot --headless --path <repo root> res://tests/ingame/main.tscn
##   godot --headless --path <repo root> res://tests/ingame/main.tscn -- --suite bsp_query

const SUITE_DIR := "res://tests/ingame/suites"


func _ready() -> void:
	# A missing extension is the one failure that would otherwise look like a pile of
	# unrelated parse errors, so say it plainly and stop.
	if not ClassDB.class_exists("GoldSrcBSP"):
		_bail("GoldSrcBSP missing — the goldsrc GDExtension did not load. Build it with ./build.sh")
		return

	var args := OS.get_cmdline_user_args()
	var only := ""
	var si := args.find("--suite")
	if si != -1 and si + 1 < args.size():
		only = args[si + 1]

	var paths := _discover(only)
	if paths.is_empty():
		_bail("no suites found in %s%s" % [SUITE_DIR, "" if only.is_empty() else " matching '%s'" % only])
		return

	var total_checks := 0
	var total_failures := 0

	for path in paths:
		var suite: RefCounted = (load(path) as GDScript).new()
		suite.run()
		suite.cleanup()

		var fails: Array = suite.failures()
		var count: int = suite.check_count()
		total_checks += count
		total_failures += fails.size()

		# A suite that ran zero checks is a silent pass — the failure mode where a file is
		# added but never wired up (a missing `run()` override, an early return) and the
		# runner cheerfully reports green. Treat it as a failure of the suite itself.
		if count == 0:
			print("FAIL %s: ran 0 checks — is run() overridden?" % suite.suite_name())
			total_failures += 1
			continue

		if fails.is_empty():
			print("PASS %s (%d checks)" % [suite.suite_name(), count])
		else:
			print("FAIL %s (%d of %d checks failed)" % [suite.suite_name(), fails.size(), count])
			for f in fails:
				print("  FAIL: %s" % f)

	print("")
	if total_failures == 0:
		print("ingame: %d checks passed across %d suite(s)" % [total_checks, paths.size()])
		get_tree().quit(0)
	else:
		print("ingame: %d of %d checks failed across %d suite(s)" % [total_failures, total_checks, paths.size()])
		get_tree().quit(1)


## Sorted so the report order is stable run to run.
func _discover(only: String) -> Array[String]:
	var out: Array[String] = []
	var dir := DirAccess.open(SUITE_DIR)
	if dir == null:
		return out
	for f in dir.get_files():
		# A bare --path run reads scripts straight off disk, but an exported/imported project
		# would list them as .remap — accept both so this doesn't depend on the import state.
		var suite_file := f.trim_suffix(".remap")
		if not suite_file.begins_with("suite_") or not suite_file.ends_with(".gd"):
			continue
		if not only.is_empty() and suite_file != "suite_%s.gd" % only:
			continue
		out.append("%s/%s" % [SUITE_DIR, suite_file])
	out.sort()
	return out


func _bail(msg: String) -> void:
	push_error("[ingame runner] %s" % msg)
	print("FAIL: %s" % msg)
	get_tree().quit(1)
