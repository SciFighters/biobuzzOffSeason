#!/usr/bin/env python3
"""Convert static Pedro Java path definitions to visualizer .pp JSON.

Usage: python3 tools/java_to_pp.py Paths.java [-o paths.pp] [--path name]
Supports BezierLine/BezierCurve builders and line/curve/path expressions,
Pose/Point variables, PoseFactory.degrees(), arithmetic constants, and
constant/linear/tangential headings. Java angles are radians; .pp uses degrees.
Paths are read in declaration order, not autonomous execution order. This is
a static converter, not a Java interpreter. Dynamic expressions, callbacks,
piecewise headings, factory transforms, and unsupported modifiers fail.
"""

import argparse
import ast
import json
import math
import operator
from pathlib import Path
import re


def split_args(text):
    """Split commas only outside nested parentheses."""
    result, start, depth = [], 0, 0
    for i, char in enumerate(text):
        depth += (char == '(') - (char == ')')
        if depth < 0:
            raise ValueError('Unbalanced parentheses')
        if char == ',' and depth == 0:
            result.append(text[start:i].strip())
            start = i + 1
    if depth:
        raise ValueError('Unbalanced parentheses')
    if text[start:].strip():
        result.append(text[start:].strip())
    return result


def call(text):
    """Read one Java call and return its name, arguments, and suffix."""
    match = re.match(r'(?:new\s+)?([\w.]+)\s*\(', text.strip())
    if not match:
        raise ValueError(f'Expected a call: {text}')
    text = text.strip()
    depth = 1
    for i in range(match.end(), len(text)):
        depth += (text[i] == '(') - (text[i] == ')')
        if depth == 0:
            return match[1], split_args(text[match.end():i]), text[i + 1:].strip()
    raise ValueError(f'Unclosed call: {text}')


class Converter:
    def __init__(self, source):
        # Preserve literals while removing comments so comment examples are ignored.
        self.source = re.sub(
            r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|//[^\n]*|/\*[\s\S]*?\*/',
            lambda m: ' ' if m[0].startswith(('//', '/*')) else m[0], source)
        self.values = dict(re.findall(
            r'\b(?:double|float|int|Pose|Point)\s+(\w+)\s*=\s*([^;]+);', self.source))
        self.factories = dict(re.findall(
            r'\bPoseFactory\s+(\w+)\s*=\s*([^;]+);', self.source))

    def number(self, text, seen=()):
        text = text.strip().removeprefix('this.')
        if text in self.values:
            if text in seen:
                raise ValueError(f'Circular constant: {text}')
            return self.number(self.values[text], (*seen, text))
        text = re.sub(r'(?<=\d)[dDfF]\b', '', text)
        try:
            node = ast.parse(text, mode='eval').body
        except SyntaxError as exc:
            raise ValueError(f'Unsupported number: {text}') from exc

        def evaluate(n):
            if isinstance(n, ast.Constant) and type(n.value) in (int, float):
                return n.value
            if isinstance(n, ast.Name):
                if n.id not in self.values or n.id in seen:
                    raise ValueError(f'Unknown or circular constant: {n.id}')
                return self.number(self.values[n.id], (*seen, n.id))
            if isinstance(n, ast.Attribute) and isinstance(n.value, ast.Name):
                if n.value.id == 'Math' and n.attr == 'PI':
                    return math.pi
            operations = {ast.Add: operator.add, ast.Sub: operator.sub,
                          ast.Mult: operator.mul, ast.Div: operator.truediv}
            if isinstance(n, ast.BinOp) and type(n.op) in operations:
                return operations[type(n.op)](evaluate(n.left), evaluate(n.right))
            if isinstance(n, ast.UnaryOp) and isinstance(n.op, (ast.UAdd, ast.USub)):
                return evaluate(n.operand) * (-1 if isinstance(n.op, ast.USub) else 1)
            if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute):
                if isinstance(n.func.value, ast.Name) and n.func.value.id == 'Math':
                    functions = {'toRadians': math.radians, 'toDegrees': math.degrees}
                    if n.func.attr in functions and len(n.args) == 1 and not n.keywords:
                        return functions[n.func.attr](evaluate(n.args[0]))
            raise ValueError(f'Unsupported numeric expression: {text}')

        value = float(evaluate(node))
        if not math.isfinite(value):
            raise ValueError(f'Non-finite number: {text}')
        return value

    def point(self, text, seen=()):
        text = text.strip().removeprefix('this.')
        if text in self.values:
            if text in seen:
                raise ValueError(f'Circular point: {text}')
            return self.point(self.values[text], (*seen, text))
        name, args, tail = call(text)
        if tail:
            raise ValueError(f'Unsupported point transform: {text}')
        if name == 'Pose.zero' and not args:
            return {'x': 0, 'y': 0, 'headingDeg': 0}
        degrees = False
        if name.endswith('.of') and name[:-3] in self.factories:
            factory = re.sub(r'\s+', '', self.factories[name[:-3]])
            if factory not in ('PoseFactory.degrees()', 'PoseFactory.radians()'):
                raise ValueError(f'Unsupported PoseFactory: {factory}')
            degrees = factory == 'PoseFactory.degrees()'
        elif name not in ('Pose', 'Point'):
            raise ValueError(f'Unsupported point: {text}')
        if name == 'Point' and len(args) == 1:
            return self.point(args[0], seen)
        if not 2 <= len(args) <= 3:
            raise ValueError(f'Expected 2 or 3 point arguments: {text}')
        point = {'x': self.number(args[0]), 'y': self.number(args[1])}
        if name == 'Point':
            if len(args) == 3 and args[2] != 'Point.CARTESIAN':
                raise ValueError('Only Point.CARTESIAN coordinates are supported')
        else:
            heading = self.number(args[2]) if len(args) == 3 else 0
            point['headingDeg'] = heading if degrees else math.degrees(heading)
        return point

    def angle(self, text):
        try:
            return math.degrees(self.number(text))
        except ValueError:
            point = self.point(text)
            if 'headingDeg' not in point:
                raise ValueError(f'Heading requires a Pose or radians: {text}')
            return point['headingDeg']

    def expression(self, text):
        name, args, tail = call(text)
        if name.endswith('.pathBuilder') and not args:
            segments = []
        elif name in ('path', 'Paths.path'):
            segments = [s for arg in args for s in self.expression(arg)]
        elif name in ('BezierLine', 'BezierCurve', 'line', 'curve', 'Paths.line', 'Paths.curve'):
            points = [self.point(arg) for arg in args]
            is_line = name in ('BezierLine', 'line', 'Paths.line')
            if (is_line and len(points) != 2) or (not is_line and len(points) < 3):
                raise ValueError(f'Wrong number of points for {name}')
            segments = [{
                '_start': points[0], 'kind': 'atomic',
                'endPoint': {k: points[-1][k] for k in ('x', 'y')},
                'controlPoints': [{k: p[k] for k in ('x', 'y')} for p in points[1:-1]],
                'heading': {'type': 'tangential', 'reverse': False},
                'color': '#ffc516',
            }]
        elif name == 'Path' and len(args) == 1:
            segments = self.expression(args[0])
        else:
            raise ValueError(f'Unsupported path expression: {text}')
        while tail:
            if not tail.startswith('.'):
                raise ValueError(f'Unsupported path suffix: {tail}')
            method, args, tail = call(tail[1:])
            if method == 'addPath' and len(args) == 1:
                segments.extend(self.expression(args[0]))
                continue
            if method == 'build' and not args:
                if tail:
                    raise ValueError('Modifiers after build() are unsupported')
                continue
            if not segments:
                raise ValueError(f'{method} has no path')
            if method in ('constant', 'setConstantHeadingInterpolation') and len(args) == 1:
                heading = {'type': 'constant', 'degrees': self.angle(args[0])}
            elif method in ('linear', 'setLinearHeadingInterpolation') and len(args) == 2:
                heading = {'type': 'linear', 'startDeg': self.angle(args[0]),
                           'endDeg': self.angle(args[1])}
            elif method in ('tangent', 'reverseTangent', 'setTangentHeadingInterpolation') and not args:
                heading = {'type': 'tangential', 'reverse': method == 'reverseTangent'}
            elif method == 'setReversed' and args in (['true'], ['false']):
                if segments[-1]['heading']['type'] != 'tangential':
                    raise ValueError('setReversed requires tangential heading')
                segments[-1]['heading']['reverse'] = args[0] == 'true'
                continue
            else:
                raise ValueError(f'Unsupported modifier: {method}({", ".join(args)})')
            if name in ('path', 'Paths.path') and len(segments) > 1:
                raise ValueError('Group-level headings are unsupported; set each segment heading')
            segments[-1]['heading'] = heading
        return segments

    def convert(self, selected=None):
        # Extract declarations and exported Path methods without interpreting Java control flow.
        pattern = (r'\b(?:Path|PathChain)\s+(\w+)\s*\(\s*\)\s*\{\s*return\s+([^;]+);'
                   r'|\b(\w+)\s*=\s*((?:[\w.]+\.pathBuilder\s*\(|new\s+(?:Path|BezierLine|BezierCurve)\s*\(|(?:Paths\.)?(?:line|curve|path)\s*\()[^;]+);')
        segments = []
        for match in re.finditer(pattern, self.source):
            name, expr = (match[1], match[2]) if match[1] else (match[3], match[4])
            if selected and name != selected:
                continue
            if re.search(r'\b' + re.escape(name) + r'\s*\.\s*(?:set\w+|addPath)\s*\(', self.source):
                raise ValueError(f'{name}: separate path mutations are unsupported; use chained modifiers')
            try:
                paths = self.expression(expr)
            except (ValueError, ZeroDivisionError, OverflowError) as exc:
                raise ValueError(f'{name}: {exc}') from exc
            for i, segment in enumerate(paths):
                segment['name'] = name if len(paths) == 1 else f'{name}_{i + 1}'
                segments.append(segment)
        if not segments:
            raise ValueError('No supported path definitions found' + (f' for {selected!r}' if selected else ''))
        for previous, current in zip(segments, segments[1:]):
            if any(not math.isclose(previous['endPoint'][k], current['_start'][k], abs_tol=1e-6)
                   for k in ('x', 'y')):
                raise ValueError(f'Discontinuous paths at {current["name"]}; use --path NAME to select one definition')
        start = dict(segments[0]['_start'])
        heading = segments[0]['heading']
        if heading['type'] == 'constant':
            start['headingDeg'] = heading['degrees']
        elif heading['type'] == 'linear':
            start['headingDeg'] = heading['startDeg']
        else:
            target = next((p for p in [*segments[0]['controlPoints'], segments[0]['endPoint']]
                           if p['x'] != start['x'] or p['y'] != start['y']), start)
            start['headingDeg'] = math.degrees(math.atan2(target['y'] - start['y'], target['x'] - start['x']))
            if heading['reverse']:
                start['headingDeg'] += 180
        for i, segment in enumerate(segments):
            segment.pop('_start')
            segment['id'] = f'path-{i + 1}'
        return {'version': '1.5.0', 'startPoint': start, 'lines': segments, 'shapes': []}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('input', type=Path, help='Java source file')
    parser.add_argument('-o', '--output', type=Path, help='Output file (default: input with .pp extension)')
    parser.add_argument('--path', help='Convert only this path variable or method')
    args = parser.parse_args()
    output = args.output or args.input.with_suffix('.pp')
    if output.resolve() == args.input.resolve():
        parser.error('Output must not overwrite Java input')
    try:
        result = Converter(args.input.read_text()).convert(args.path)
        output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    except (OSError, ValueError, ZeroDivisionError, OverflowError) as exc:
        parser.exit(1, f'Error: {exc}\n')
    print(f'Wrote {output}: {len(result["lines"])} segment(s), source declaration order')


if __name__ == '__main__':
    main()
