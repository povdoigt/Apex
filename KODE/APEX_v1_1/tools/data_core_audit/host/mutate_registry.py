"""Host mutation check of data_topic guards: subscriber registry and cursor
check (sequential build).

Each mutant is a textual change of data_topic.c. The host suites (CB, DT, DP)
are rebuilt against the mutant; a mutant is "killed" when at least one
non-ISR case fails. A surviving mutant means a guard no test depends on:
either the tests are too weak, or the guard is redundant (to be argued).

Usage: python mutate_registry.py
"""
import os
import re
import shutil
import subprocess
import sys

H = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.normpath(os.path.join(H, '../../../UserLibraries/utils/data_topic/Core/Src/data_topic.c'))
MUT = os.path.join(H, 'mut')
os.makedirs(MUT, exist_ok=True)

MUTANTS = [
    ('attach_no_drop',
     '        for (size_t i = 0u; i < DATA_TOPIC_MAX_SUBS; i++) {\n            dt_drop_invalid_locked(topic, i);\n        }\n',
     ''),
    ('valid_no_topic',
     '    return (sub->attached != 0) && (sub->topic == topic);',
     '    return (sub->attached != 0);'),
    ('valid_no_attached',
     '    return (sub->attached != 0) && (sub->topic == topic);',
     '    return (sub->topic == topic);'),
    ('detach_no_fault',
     '        topic->list_faults++;               /* absent :',
     '        /* MUTANT */;               /* absent :'),
    ('detach_count_always',
     '    if (slot < DATA_TOPIC_MAX_SUBS) {\n        dt_unregister_locked(topic, slot);\n    } else {',
     '    if (1) {\n        if (slot < DATA_TOPIC_MAX_SUBS) topic->subs[slot] = NULL;\n        topic->sub_count--;\n    } else {'),
    ('free_all_valid',
     '        const bool valid = (sub != NULL) && dt_sub_valid(sub, topic);',
     '        const bool valid = (sub != NULL);'),
    ('drop_no_fault',
     '        dt_unregister_locked(topic, slot);\n        topic->list_faults++;\n',
     '        dt_unregister_locked(topic, slot);\n'),
    ('slot_search_short',
     '    for (size_t i = 0u; i < DATA_TOPIC_MAX_SUBS; i++) {\n        if (topic->subs[i] == sub) {',
     '    for (size_t i = 0u; i < DATA_TOPIC_MAX_SUBS - 1u; i++) {\n        if (topic->subs[i] == sub) {'),
    ('attach_no_count',
     '            topic->subs[slot] = sub;\n            topic->sub_count++;\n',
     '            topic->subs[slot] = sub;\n'),
    ('init_no_clear',
     '    for (size_t i = 0u; i < DATA_TOPIC_MAX_SUBS; i++) {\n        topic->subs[i] = NULL;\n    }\n',
     ''),
    ('reset_keeps_topic',
     '    sub->attached = 0;\n    sub->topic    = NULL;\n',
     '    sub->attached = 0;\n'),
    ('cursor_unchecked',
     '    const bool consistent = (lag <= stored) &&\n'
     '                            (sub->tail == (cb->head + cb->capacity - (size_t)lag) % cb->capacity);',
     '    const bool consistent = (lag <= stored);'),
    ('unregister_no_null',
     '    topic->subs[slot] = NULL;\n    if (topic->sub_count > 0u) {',
     '    if (topic->sub_count > 0u) {'),
]


def run(dt_src, exe):
    build = os.path.join(H, 'build.sh')
    # Git Bash explicitly: a bare 'bash' resolves to WSL's on Windows.
    p = subprocess.run([shutil.which('bash') or 'bash', build, dt_src, exe], capture_output=True, text=True, encoding='utf-8', errors='replace')
    return p.returncode, p.stdout + p.stderr


def killed_by(out):
    fails = []
    for line in out.splitlines():
        m = re.match(r'\s*\[FAIL\]\s+(\S+\s+\S+)', line)
        if m and 'ISR TIM5' not in line:
            fails.append(m.group(1))
    return fails


def main():
    base = open(SRC, encoding='utf-8').read()
    survivors = 0
    for name, old, new in MUTANTS:
        if base.count(old) != 1:
            print(f'[{name}] PATTERN NOT FOUND ({base.count(old)} matches)')
            survivors += 1
            continue
        path = os.path.join(MUT, f'dt_{name}.c')
        open(path, 'w', encoding='utf-8', newline='\n').write(base.replace(old, new))
        rc, out = run(path, os.path.join(MUT, f't_{name}.exe'))
        if 'error:' in out:
            print(f'[{name}] BUILD ERROR\n{out[:800]}')
            survivors += 1
            continue
        fails = killed_by(out)
        if rc != 0 or not re.search(r'^\d+ FAIL$', out, re.M):
            fails.append(f'CRASH (rc={rc})')       # a crash kills the mutant too
        status = 'KILLED' if fails else 'SURVIVED'
        if not fails:
            survivors += 1
        print(f'[{name}] {status} by: {", ".join(fails) if fails else "-"}')
    print(f'{survivors} survivor(s) out of {len(MUTANTS)}')


if __name__ == '__main__':
    main()
