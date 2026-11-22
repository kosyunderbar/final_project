from functools import reduce
def frame(body: str) -> str:
    """body = 종류 글자 + 내용 (예: 'C,re=42,ok=1')"""
    cs = reduce(lambda a, b: a ^ b, body.encode(), 0)
    return f"${body}*{cs:02X}"
def check(line: str) -> bool:
    line = line.strip()
    if not line.startswith("$") or len(line) < 5 or line[-3] != "*": return False
    try: return frame(line[1:-3]).upper() == line.upper()
    except ValueError: return False
if __name__ == "__main__":
    for want in ['$R{"n":42,"k":"DONE"}*7B', '$C,re=42,ok=1*67',
                 '$T{"n":41,"k":"TAG","uid":"04A1B2C3"}*36', '$H{"up":123,"st":"WORK"}*51',
                 '$S,st=WORK,lamp=Yb,fault=0*35', '$X,id=7,in=S1,v=1*51']:
        got = frame(want[1:-3]); print(got, "OK" if got == want and check(want) else "FAIL")
    print(frame("L,up=0"), frame("L,up=1"))
