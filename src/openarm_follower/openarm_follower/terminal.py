"""한글이 섞인 터미널 표와 제자리 상태 줄.

한글 같은 전각 문자는 한 글자가 두 칸을 차지해서, 글자 수로 맞춘 표는 어긋나고
글자 수로 자른 상태 줄은 터미널 폭을 넘는다.
"""

import math
import shutil
import sys
import unicodedata


def cols(text):
    """터미널 표시 폭."""
    return sum(2 if unicodedata.east_asian_width(c) in ('W', 'F') else 1 for c in text)


def pad(text, width):
    """표시 폭 width 칸까지 뒤를 공백으로 채운 text. 넘치면 한 칸만 띄운다."""
    return text + ' ' * max(1, width - cols(text))


def fit(text, columns):
    """columns 칸보다 짧게 자른 text.

    상태 줄은 \\r 로 같은 줄을 덮어쓴다. 터미널 폭을 넘으면 줄이 접혀 \\r 이 마지막
    조각만 덮고, 갱신할 때마다 화면이 한 줄씩 밀려 올라간다. 그래서 마지막 칸을
    비워 두고 자른다.
    """
    out, used = [], 0
    for c in text:
        width = cols(c)
        if used + width > columns - 1:
            break
        out.append(c)
        used += width
    return ''.join(out)


def status(line):
    """상태 줄을 제자리에 덮어쓴다."""
    columns = shutil.get_terminal_size((80, 24)).columns
    sys.stdout.write('\r' + fit(line, columns) + '\033[K')
    sys.stdout.flush()


def deg(rad):
    """부호 붙은 도 단위 표기. 예: +12.5°."""
    return f'{math.degrees(rad):+.1f}°'
