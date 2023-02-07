"""
A very quick and dirty tool to simulate CAN filters
for socket-can in C. Append test_filter(ID, MASK) to
the filter_list in order for it to be checked.
"""

start   = 0x000
end     = 0xFFF

def main():
    filter_list = []
    filter_list.append(test_filter(0x020,0xff8))
    filter_list.append(test_filter(0x028,0xffc))

    print("#"*64)
    for idx,f in enumerate(filter_list):
        print(f"Filter #{idx} || ID: {format(f.can_id,'#05x')}\
        MASK: {(format(f.can_mask,'#05x'))}")
    print(f"Sample Range: {format(start,'#05x')} --> {format(end,'#05x')}\n{'#'*64}")


    for i in range(start,end+1):
        accept_frame = False
        for f in filter_list:
            f_id = (i & f.can_mask)
            if f_id == f.can_id:
                accept_frame = True
        if accept_frame:
            print(f"ID:{format(i,'#05x')} ... {format(i, '#014b')}\
            || Accepted?: {accept_frame}");
        


class test_filter():
    def __init__(self, id, mask):
        self.can_id     = id
        self.can_mask   = mask
    

if __name__ == "__main__":
    main()
