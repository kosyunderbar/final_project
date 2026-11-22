import html,sys,os
here=os.path.dirname(os.path.abspath(__file__)); root=os.path.dirname(here)
rd=lambda p: open(os.path.join(root,p),encoding="utf-8").read()
t=rd("src/book.tpl.html")
for k,f in [("MODEM","code/modem/modem.ino"),("TAG","code/tag/tag.ino"),("CSC","code/cs.c"),("CSPY","code/cs.py")]:
    assert "{{%s}}"%k in t; t=t.replace("{{%s}}"%k, html.escape(rd(f).rstrip(),quote=False))
assert "{{" not in t.split("<script>")[0]
open(os.path.join(root,"src/ESP32_교본.artifact.html"),"w",encoding="utf-8").write(t)
full='<!doctype html>\n<html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">\n<style>html{color-scheme:light}body{margin:0}img{max-width:100%}[hidden]{display:none!important}</style>\n'+t+'\n</html>\n'
full=full.replace('<div class="wrap">','</head><body>\n<div class="wrap">',1).replace('</script>\n\n</html>','</script>\n</body></html>')
open(os.path.join(root,"ESP32_교본.html"),"w",encoding="utf-8").write(full)
print("built",len(t))
