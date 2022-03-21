import clang.cindex

def traverse_tree(node, level):
    if node.location.line == 45 and node.kind == clang.cindex.CursorKind.DECL_STMT:
        print("stop")
    if not list(node.get_children()):
        print(f"############################## {level}") 
    for child_node in node.get_children():
        print(child_node.location.line, child_node.location.column, child_node.kind, level)
        traverse_tree(child_node, level+1)

c_fn = './circular_controller_threelanes_3cars_explicit2.c'

index = clang.cindex.Index.create()
translation_unit = index.parse(c_fn, args=['-x','c','-std=c99'])

node = translation_unit.cursor

traverse_tree(node, 0)