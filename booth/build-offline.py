#!/usr/bin/env python3
"""parking.html(아티팩트용 조각) -> index.html(단독 실행본).

아티팩트로 올릴 때는 claude.ai 가 doctype·charset·viewport 를 붙여 주지만,
부스 노트북에서 더블클릭하거나 S3 에 올릴 때는 그게 없다. 그 껍데기를 여기서 씌운다.
펌웨어 wasm 은 이미 parking.html 안에 base64 로 들어 있어 따로 챙길 파일이 없다.

    python3 build-offline.py
"""
import pathlib, re, sys

HERE = pathlib.Path(__file__).parent
src = (HERE / 'parking.html').read_text(encoding='utf-8')

title = re.search(r'<title>(.*?)</title>', src, re.S)
title = title.group(1).strip() if title else '로봇 말만 믿고 주차하기'

# 조각은 <meta>·<title>·<link>·<style> 다음에 마크업, 그 다음 <script> 순서다.
# 스크립트가 마크업보다 먼저 돌면 getElementById 가 null 이 되므로 그 순서를 지켜 나눈다.
cut = src.index('<div id="app">')
head_part, body_part = src[:cut].rstrip(), src[cut:].strip()

out = f"""<!doctype html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<meta name="description" content="바깥을 못 보는 로봇이 제 위치를 얼마나 모르게 되는지 60초 만에 보여 주는 부스용 게임">
<style>
  :root {{ color-scheme: dark; }}
  html, body {{ margin: 0; }}
  img {{ max-width: 100%; }}
  [hidden] {{ display: none !important; }}
</style>
{head_part}
</head>
<body>
{body_part}
</body>
</html>
"""

dst = HERE / 'index.html'
dst.write_text(out, encoding='utf-8')
print(f'{dst}  ·  {len(out):,} chars  ·  제목 "{title}"')
print('더블클릭해서 열리면 됩니다. 인터넷 없어도 동작합니다.')
