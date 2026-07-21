extends RefCounted
## Base for the in-engine suites.
##
## Deliberately parallel to goldnet-godot's tests/gdscript/suite.gd — same assertion names
## and same reporting contract, so the two repos' harnesses read alike. Keep them in step.
##
## No `class_name` on purpose: global class names resolve from a cache the editor writes
## during an import pass, which a bare `--headless --path` run doesn't perform. Suites
## `extends "res://tests/ingame/suite.gd"` by path so this works from a clean checkout with
## no editor step. The classes under test (GoldSrcBSP and friends) are GDExtension classes
## registered at engine start, not script classes, so those resolve as bare identifiers.
##
## Suites override `run()` and call the assertions below. Assertions accumulate rather than
## abort, so one run reports everything wrong instead of only the first failure.

## The one real map in the repo. Every geometry assertion is anchored to this file, so
## replacing it means re-deriving the numbers that reference it.
const MAP_PATH := "res://maps/ww_2fort.bsp"
const SPRITE_PATH := "res://sprites/fire.spr"

## GoldSrc leaf contents. Not exported to GDScript by the extension, so they are restated
## here; they are format constants and will not move.
const CONTENTS_EMPTY := -1
const CONTENTS_SOLID := -2
const CONTENTS_WATER := -3
const CONTENTS_SKY := -6

var _failures: Array[String] = []
var _checks: int = 0
var _to_free: Array[Node] = []


## Override with the suite's tests.
func run() -> void:
	pass


## Human-readable suite name, used in the report. Defaults to the script's filename.
func suite_name() -> String:
	return (get_script() as GDScript).resource_path.get_file().get_basename()


# --- helpers ---

## Node-derived GoldSrc classes are not refcounted, so anything a suite creates has to be
## freed or the run ends with "ObjectDB instances leaked at exit". Hand them to this and
## the runner disposes of them after run() returns.
func track(n: Node) -> Node:
	_to_free.append(n)
	return n


## Loads the shared map into a tracked GoldSrcBSP, failing the suite if it will not parse.
## Returns null on failure so callers can bail out rather than assert against a blank node.
func load_map() -> GoldSrcBSP:
	var bsp := GoldSrcBSP.new()
	track(bsp)
	var err := bsp.load_bsp(MAP_PATH)
	if err != OK:
		fail("could not load %s (error %d)" % [MAP_PATH, err])
		return null
	return bsp


func cleanup() -> void:
	for n in _to_free:
		if is_instance_valid(n):
			n.free()
	_to_free.clear()


# --- assertions ---

func check(cond: bool, msg: String) -> bool:
	_checks += 1
	if not cond:
		_failures.append(msg)
	return cond


func check_eq(got: Variant, want: Variant, msg: String) -> bool:
	return check(got == want, "%s (got %s, want %s)" % [msg, got, want])


## Inclusive on both ends.
func check_between(got: float, lo: float, hi: float, msg: String) -> bool:
	return check(got >= lo and got <= hi,
		"%s (got %s, want %s..%s)" % [msg, got, lo, hi])


func check_near(got: float, want: float, tol: float, msg: String) -> bool:
	return check(absf(got - want) <= tol,
		"%s (got %f, want %f, tol %f)" % [msg, got, want, tol])


func check_near_v(got: Vector3, want: Vector3, tol: float, msg: String) -> bool:
	return check(got.distance_to(want) <= tol,
		"%s (got %.4v, want %.4v, tol %f)" % [msg, got, want, tol])


func fail(msg: String) -> void:
	_checks += 1
	_failures.append(msg)


# --- harness plumbing ---

func failures() -> Array[String]:
	return _failures


func check_count() -> int:
	return _checks
