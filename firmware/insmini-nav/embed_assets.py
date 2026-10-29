Import('env')
import gzip, pathlib
project=pathlib.Path(env['PROJECT_DIR'])
names={'index.html':'index_html','app.js':'app_js','style.css':'style_css'}
parts=['#pragma once\n#include <Arduino.h>\n']
for name,symbol in names.items():
    data=gzip.compress((project/'web'/name).read_bytes(),compresslevel=9,mtime=0)
    parts.append('static const uint8_t '+symbol+'[] PROGMEM={'+','.join(map(str,data))+'};\n')
out=project/'src/web_assets.h';text=''.join(parts)
if not out.exists() or out.read_text()!=text:out.write_text(text,encoding='utf8')
