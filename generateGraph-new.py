#Accidentally uploaded old code and left computer with newest code at home!!!! 
#This is quite old! It only computes a path, but doesn't distingusih between guards and resets

from cgitb import reset
import clang.cindex
import typing
import json
import sys
from typing import List, Tuple
import re 
#from cfg import CFG
#import clang.cfg


#index = clang.cindex.Index.create()
#translation_unit = index.parse('my_source.c', args=['-std=c99'])

#print all the cursor types
#for i in translation_unit.get_tokens(extent=translation_unit.cursor.extent):
#    print (i.kind)

#get the structs
def filter_node_list_by_node_kind(
    nodes: typing.Iterable[clang.cindex.Cursor],
    kinds: list
) -> typing.Iterable[clang.cindex.Cursor]:
    result = []
    for i in nodes:
        if i.kind in kinds:
            result.append(i)
    return result

#exclude includes
def filter_node_list_by_file(
    nodes: typing.Iterable[clang.cindex.Cursor],
    file_name: str
) -> typing.Iterable[clang.cindex.Cursor]:
    result = []
    for i in nodes:
        if i.location.file.name == file_name:
            result.append(i)
    return result


#all_classes = filter_node_list_by_node_kind(translation_unit.cursor.get_children(), [clang.cindex.CursorKind.ENUM_DECL, clang.cindex.CursorKind.STRUCT_DECL])
#for i in all_classes:
#    print (i.spelling)

#captured fields in classes
def is_exposed_field(node):return node.access_specifier == clang.cindex.AccessSpecifier.PUBLIC
def find_all_exposed_fields(
    cursor: clang.cindex.Cursor
):
    result = []
    field_declarations = filter_node_list_by_node_kind(cursor.get_children(), [clang.cindex.CursorKind.FIELD_DECL])
    for cursor in field_declarations:
        if list(cursor.get_tokens())[0].spelling == "enum":
            continue
        if not is_exposed_field(cursor):
            continue
        result.append(cursor.displayname)
    return result


def populate_field_list_recursively(class_name: str,class_field_map,class_inheritance_map):
    field_list = class_field_map.get(class_name)
    if field_list is None:
        return []
    baseClasses = class_inheritance_map[class_name]
    for i in baseClasses:
        field_list = populate_field_list_recursively(i.spelling) + field_list
    return field_list
#rtti_map = {}



def get_state_variables(translation_unit):
    rtti_map = {}
    source_nodes = filter_node_list_by_file(translation_unit.cursor.get_children(), translation_unit.spelling)
    all_classes = filter_node_list_by_node_kind(source_nodes, [clang.cindex.CursorKind.STRUCT_DECL])
    class_inheritance_map = {}
    class_field_map = {}
    for i in all_classes:
        bases = []
        for node in i.get_children():
            if node.kind == clang.cindex.CursorKind.CXX_BASE_SPECIFIER:
                referenceNode = node.referenced
                bases.append(node.referenced)
        class_inheritance_map[i.spelling] = bases
    for i in all_classes:
        fields = find_all_exposed_fields(i)
        class_field_map[i.spelling] = fields
    #print("foo")
    for class_name, class_list in class_inheritance_map.items():
        rtti_map[class_name] = populate_field_list_recursively(class_name,class_field_map,class_inheritance_map)
    for class_name, field_list in rtti_map.items():
        rendered_fields = []
        for f in field_list:
            rendered_fields.append(f)
    return rendered_fields



#def populate_field_list_recursively(class_name: str):
#    field_list = class_field_map.get(class_name)
#    if field_list is None:
#        return []
#    baseClasses = class_inheritance_map[class_name]
#    for i in baseClasses:
#        field_list = populate_field_list_recursively(i.spelling) + field_list
#    return field_list
#rtti_map = {}

#def get_state_variables():
#    print("foo")
#    for class_name, class_list in class_inheritance_map.items():
#        rtti_map[class_name] = populate_field_list_recursively(class_name)
#    for class_name, field_list in rtti_map.items():
#        rendered_fields = []
#        for f in field_list:
#            rendered_fields.append(f)
#    return rendered_fields



#how can we get the structure with the if statements
#need to get ahold of if statement, then using if statement methods, we can get the inner statement


#all_ifs = filter_node_list_by_node_kind(translation_unit.cursor.get_children(), [clang.cindex.CursorKind.IF_STMT])
#for i in all_ifs:
#    print (i.spelling)

#print(translation_unit.spelling)

def find_ifstmt(node):
    if node.kind.is_statement():
        #ref_node = clang.cindex.Cursor_ref(node)
        #print ("found %s"" % node.location.line)
        print(node.displayname)
        print(node.location.line)
    for c in node.get_children():
        find_ifstmt(c)

#finds the if statement, but cant get anything out of it
#find_ifstmt(translation_unit.cursor)


def rectree(node, indent):
    print("%s item %s of kind %s" % (indent, node.spelling, node.kind))
    for c in node.get_children():
        rectree(c, indent + "  ")
#goes through every part of code but doesn't give info about non variable names
#rectree(translation_unit.cursor, "")

#CursorKind.IF_STMT

#given the if statement as node, visit each of the else if statements
def visit_elif(node):
    print("visiting inner elseif statement")
    for i in node.get_children():
        #print(i.kind)
        #print(i.spelling)
        if i.kind == clang.cindex.CursorKind.IF_STMT or i.kind == clang.cindex.CursorKind.COMPOUND_STMT:
            print("found el-if of compound")
            print(i.spelling)
            visit_elif(i)
    print("finished elseif statement")    
#if (node.hasElseStorage()):
        #print("foo")


#TODO, but a method to get us the info from an if statemen
def visitif(node):
    print(node.spelling)

def visit_inner_if(node):
    for i in node.get_children():
        if i.kind == clang.cindex.CursorKind.IF_STMT:
            print("inner if %s" % i.kind)
            visit_elif(node)
        else:
            visit_inner_if(i)

def get_cursor_id(cursor, cursor_list = []):
    if cursor is None:
        return None

    # FIXME: This is really slow. It would be nice if the index API exposed
    # something that let us hash cursors.
    for i,c in enumerate(cursor_list):
        if cursor == c:
            return i
    cursor_list.append(cursor)
    return len(cursor_list) - 1

def get_info(node, depth=0):
    children = [get_info(c, depth+1)
                    for c in node.get_children()]
    return { 'id' : get_cursor_id(node),
             'kind' : node.kind,
             'usr' : node.get_usr(),
             'spelling' : node.spelling,
             'location' : node.location,
             'extent.start' : node.extent.start,
             'extent.end' : node.extent.end,
             'is_definition' : node.is_definition(),
             'definition id' : get_cursor_id(node.get_definition())}
             #'children' : children }

def code_from_cursor(cursor):
    code = []
    line = ""
    prev_token = None
    for tok in cursor.get_tokens():
        if prev_token is None:
            prev_token = tok
        prev_location = prev_token.location
        prev_token_end_col = prev_location.column + len(prev_token.spelling)
        cur_location = tok.location
        if cur_location.line > prev_location.line:
            code.append(line)
            line = " " * (cur_location.column - 1)
        else:
            if cur_location.column > (prev_token_end_col):
                line += " "
        line += tok.spelling
        prev_token = tok
    if len(line.strip()) > 0:
        code.append(line)
    return code

def code_from_cursor_simpler(cursor):
    code = ""
    for tok in cursor.get_tokens():
        code += tok.spelling
    return code

def visit_outter_if(node):
    if_statements = []
    for i in node.get_children():
        if i.kind == clang.cindex.CursorKind.IF_STMT:
            #print("Outter if statement %s" % i.kind)
            #visit_inner_if(i)
            if_statements.append(code_from_cursor(i)[0])
            #print(if_statements)
            #print(i.spelling)
            #for child in i.get_children():
            #    print(str(i.kind) + str(i.spelling))
        else:
            out_ifs = visit_outter_if(i)
            for statement in out_ifs:
                if_statements.append(statement)
    return if_statements

#visit_outter_if(translation_unit.cursor)


def get_outter_if_cursors(node):
    if_statements = []
    for i in node.get_children():
        if i.kind == clang.cindex.CursorKind.IF_STMT:
            #print("Outter if statement %s" % i.kind)
            #visit_inner_if(i)
            if_statements.append(i)
            #print(if_statements)
            #print(i.spelling)
            #for child in i.get_children():
            #    print(str(i.kind) + str(i.spelling))
        else:
            out_ifs = get_outter_if_cursors(i)
            for statement in out_ifs:
                if_statements.append(statement)
    return if_statements


def get_inner_if_cursors(node):
    if_statements = []
    for i in node.get_children():
        #print(i.kind)
        if i.kind == clang.cindex.CursorKind.IF_STMT:
            if_statements.append(i)
            #print(if_statements)
            #print(i.spelling)
            #for child in i.get_children():
            #    print(str(i.kind) + str(i.spelling))
        else:
            out_ifs = get_inner_if_cursors(i)
            for statement in out_ifs:
                if_statements.append(statement)
    return if_statements

class if_tree_node:
    def __init__(self, cursor: clang.cindex.Cursor, children_ifs = []):
        self.child_node: List(if_tree_node) = children_ifs
        self.cursor: clang.cindex.Cursor = cursor 

def find_immediate_if(cursor:clang.cindex.Cursor) -> List[if_tree_node]:
    # if cursor.kind == clang.cindex.CursorKind.IF_STMT:
    #     node = if_tree_node(cursor, [])
    #     return [node]
    # else:
    #     res = []
    #     for child_cursor in cursor.get_children():
    #         res += find_immediate_if(child_cursor)
    #     return res
    res = []
    for child_cursor in cursor.get_children():
        if child_cursor.kind == clang.cindex.CursorKind.IF_STMT or child_cursor.kind == clang.cindex.CursorKind.FOR_STMT:
            node = if_tree_node(child_cursor, [])
            res.append(node)
        else:
            res += find_immediate_if(child_cursor)
    return res

# Purge the complicated cursor tree to ones that only have ifs
# Input: cursor, the root cursor that we want to explore
#        root, the root of the if tree
# Return: List(if_tree_node), the list of immediate children if statement
def extract_if_tree(cursor:clang.cindex.Cursor) -> if_tree_node:
    node_list = []
    root = if_tree_node(None)
    root.child_node = find_immediate_if(cursor)
    for node in root.child_node:
        node_list.append(node)
    while node_list:
        child_node = node_list.pop(0)
        child_cursor = child_node.cursor
        child_node.child_node = find_immediate_if(child_cursor)
        for node in child_node.child_node:
            node_list.append(node)
    # for child_node in root.child_node:
    #     child_cursor = child_node.cursor
    #     child_node.child_node = find_immediate_if(child_cursor)
    return root

# def extract_num_mode_params(nodes: typing.Iterable[clang.cindex.Cursor]) -> int:
#     return 2

def extract_modes(root: if_tree_node) -> Tuple[List[str], int]:
    res = []
    node_list = []
    num_mode_kw = -1
    for node in root.child_node:
        if node.child_node:
            cursor = node.cursor
            condition_cursor = list(cursor.get_children())[0]
            mode_str = list(condition_cursor.get_tokens())[2].spelling
            node_list.append((node, mode_str, 1))
    while node_list:
        par_node, prev_mode_str, num_kw = node_list.pop()
        for node in par_node.child_node:
            if node.child_node:
                cursor = node.cursor
                condition_cursor = list(cursor.get_children())[0]
                mode_str = list(condition_cursor.get_tokens())[2].spelling
                node_list.append((node, prev_mode_str + ";" + mode_str, num_kw+1))
            else:
                if prev_mode_str not in res:
                    res.append(prev_mode_str)
                    num_mode_kw = max(num_mode_kw, num_kw)

    return res, num_mode_kw

def extract_transitions(root: if_tree_node):
    mode_list, num_kw = extract_modes(root)
    node_list = []
    edge = []
    guards = []
    resets = []

    for node in root.child_node:
        cursor = node.cursor
        condition_cursor = list(cursor.get_children())[0]
        mode_str = list(condition_cursor.get_tokens())[2].spelling
        node_list.append((node, mode_str))
    while node_list:
        par_node, prev_mode_str = node_list.pop()
        for node in par_node.child_node:
            if node.child_node:
                cursor = node.cursor
                condition_cursor = list(cursor.get_children())[0]
                mode_str = list(condition_cursor.get_tokens())[2].spelling
                node_list.append((node, prev_mode_str + ";" + mode_str))
            else:
                cursor = node.cursor 

                # Get the source of the edge
                edge_src = prev_mode_str 
                edge_src_idx = mode_list.index(edge_src)

                # Get the guard for the edge
                edge_guard_cursor = list(cursor.get_children())[0]
                guard_str = code_from_cursor_simpler(edge_guard_cursor)
                edge_guard = guard_str 

                # Get the destination of the edge
                compound_cursor = list(cursor.get_children())[1]
                code_cursors = list(compound_cursor.get_children())
                dest_mode_kw = []
                for i in range(num_kw):
                    guard_cursor = code_cursors[i]
                    dest_mode_kw += [list(guard_cursor.get_tokens())[2].spelling]
                edge_dest = dest_mode_kw[0]
                for i in range(1,len(dest_mode_kw)):
                    edge_dest = edge_dest+";"+dest_mode_kw[i]
                if edge_dest not in mode_list:
                    mode_list.append(edge_dest)
                edge_dest_idx = mode_list.index(edge_dest)

                # Get the resets for the edge
                edge_resets = ""
                for i in range(num_kw, len(code_cursors)):
                    reset_cursor = code_cursors[i]
                    edge_resets += (code_from_cursor_simpler(reset_cursor)+";")
                if edge_resets != "" and edge_resets[-1] == ";":
                    edge_resets=edge_resets[:-1]
                edge.append([edge_src_idx, edge_dest_idx])
                guards.append(edge_guard)
                resets.append(edge_resets)
    guards = pretty_guards(guards)
    return mode_list, edge, guards, resets                    

#input: current node, the set of guards/conditionals up to this point, the set of resets
#return: the sets of guards/resets for all the children
def traverse_tree(node, guards, resets, indent, hasIfParent):
    #print(guards)
    childrens_guards=[]
    childrens_resets=[]
    results = []
    found = []
    #print(resets)
    #if (indent== '------'):
    #    print(guards)
    #    return [guards,resets]
    #if (node.kind == clang.cindex.CursorKind.RETURN_STMT):
    #    return [guards, resets]
    for i in node.get_children():
        if i.kind == clang.cindex.CursorKind.IF_STMT: #TODO: add else statements
            temp_results = traverse_tree(i, guards, resets, indent+ "-", True)
            for item in temp_results:
                found.append(item)
            #childrens_guards.append(result_guards)
            #childrens_resets.append(result_resets)

        else:
            reset_copy = resets
            guard_copy = guards
            if hasIfParent:
                if node.kind == clang.cindex.CursorKind.IF_STMT and i.kind != clang.cindex.CursorKind.COMPOUND_STMT:
                    childrens_guards.append(code_from_cursor(i))
                    print(indent + "Guard statement")
                    print(code_from_cursor(i))
                    #found.append(code_from_cursor(i))
                    #temp_results = traverse_tree(i, guard_copy, reset_copy, indent+ "-", hasIfParent)
                    #for result in temp_results:
                    #    result.append(code_from_cursor(i))
                    #    results.append(result)
                    #if len(temp_results)== 0:
                    #    result.append([code_from_cursor(i)])
                    #    print("foo")
                if node.kind == clang.cindex.CursorKind.COMPOUND_STMT and i.kind != clang.cindex.CursorKind.DECL_STMT:
                    print(indent + "Reset statement")
                    print(code_from_cursor(i))
                    childrens_resets.append(code_from_cursor(i))
                    #found.append(code_from_cursor(i))
                    #temp_results = traverse_tree(i, guard_copy, reset_copy, indent+ "-", hasIfParent)
                    #for result in temp_results:
                    #    result.append(code_from_cursor(i))
                    #    results.append(result)
                    #if len(temp_results) == 0:
                    #    results.append([code_from_cursor(i)])
                    #    print("foo")
                #else:
            temp_results = traverse_tree(i, guard_copy, reset_copy, indent+ "-", hasIfParent)
            for item in temp_results:
                 found.append(item)
            #childrens_guards.append(results[0])
            #childrens_resets.append(results[1])
    #print(results)
    #output = []
    #if len(results) < 0:
    #    input = []
    #    input.append(childrens_results)
    #    input.append(childrens_guards)
    #    output.append(input)
    #else:
    #    for result in results:
    #        result.append(childrens_resets)
    #        result.append(childrens_guards)
    #        output.append(result)
    #print(output)
    #ASSUMPTION: lowest level is a reset!!!
    if len(found) == 0 and len(childrens_resets) > 0:
        found.append([])
    for item in found:
        for reset in childrens_resets:
        #for item in found:
            item.append([reset, 'r'])
        results.append(item)
        for guard in childrens_guards:
            item.append([guard, 'g'])

    return results

#assumption, word mode is not in states
def pretty_vertices(vertices):
    output = []
    for vertex_code in vertices:
        parts = vertex_code.split("==")
        nonwhitespace = parts[-1].split()
        if "mode" not in nonwhitespace:
            output.append(nonwhitespace[0].strip(')'))
    return output
 
def get_next_state(code):
    line = code[-2]
    parts = line.split("=")
    state = parts[-1].strip(';')
    return state.strip()

class logic_tree_node:
    def __init__(self, data, child = []):
        self.data = data 
        self.child = child

# Split a single logic string into a list of strings 
# with atomic logic expression, "(", ")", "&&", "||"
def logic_string_split(logic_string):
    res = re.split('(&&)',logic_string)

    tmp = []
    for sub_str in res:
        tmp += re.split('(\|\|)',sub_str)
    res = tmp

    tmp = []
    for sub_str in res:
        tmp += re.split('(\()',sub_str)
    res = tmp

    tmp = []
    for sub_str in res:
        tmp += re.split('(\))',sub_str)
    res = tmp

    while("" in res) :
        res.remove("")

    return res 

def generate_logic_tree(logic_string):
    # Construct expression tree from inorder expression
    # https://www.geeksforgeeks.org/program-to-convert-infix-notation-to-expression-tree/
    logic_string = "(" + logic_string + ")"
    s = logic_string_split(logic_string)
    print(s)
    stN = []
    stC = []
    p = {}
    p["&&"] = 1
    p["||"] = 1
    p[")"] = 0
    for i in range(len(s)):
        if s[i] == "(":
            stC.append(s[i])
        
        elif s[i] not in p:
            t = logic_tree_node(s[i])
            stN.append(t)
        
        elif(p[s[i]]>0):
            while (len(stC) != 0 and stC[-1] != '(' and p[stC[-1]] >= p[s[i]]):
                                # Get and remove the top element
                # from the character stack
                t = logic_tree_node(stC[-1])
                stC.pop()
 
                # Get and remove the top element
                # from the node stack
                t1 = stN[-1]
                stN.pop()
 
                # Get and remove the currently top
                # element from the node stack
                t2 = stN[-1]
                stN.pop()
 
                # Update the tree
                t.child = [t1, t2]
 
                # Push the node to the node stack
                stN.append(t)
            stC.append(s[i])
        elif (s[i] == ')'):
            while (len(stC) != 0 and stC[-1] != '('):
                # from the character stack
                t = logic_tree_node(stC[-1])
                stC.pop()
 
                # Get and remove the top element
                # from the node stack
                t1 = stN[-1]
                stN.pop()
 
                # Get and remove the currently top
                # element from the node stack
                t2 = stN[-1]
                stN.pop()
 
                # Update the tree
                t.child = [t1, t2]
 
                # Push the node to the node stack
                stN.append(t)
            stC.pop()
    t = stN[-1]
    return t

def generate_guard_string(root: logic_tree_node)->str:
    if root.data!="&&" and root.data!="||":
        return root.data
    else:
        data1 = generate_guard_string(root.child[0])
        data2 = generate_guard_string(root.child[1])
        if root.data == "&&":
            return f"And({data1},{data2})"
        elif root.data == "||":
            return f"Or({data1},{data2})"
        
#TODO: consider or??
def pretty_guards(code):
    guard_string_list = []
    for logic_string in code:
        # split_logic_string = logic_string_split(logic_string)
        # print(split_logic_string)

        root = generate_logic_tree(logic_string)
        guard_string = generate_guard_string(root)
        print(guard_string)
        guard_string_list.append(guard_string)
    # line = code[0]
    # conditional = line.strip('if').strip('{')
    # conditions = conditional.split('&&')
    # output = "And"
    # for condition in conditions: re
    #     output += condition.strip() + ","
    # output = output.strip(",")
    # return output
    return guard_string_list

#assumption: last two lines of code are reset and bracket... not idea
def pretty_resets(code):
    outstring = ""
    for index in range(0,len(code)):
       if index != 0 and index != len(code)-1 and index != len(code)-2:
        outstring += code[index].strip().strip('\n')
    return outstring.strip(';')

def traverse_tree(root: if_tree_node, level = 0):
    if root.cursor is not None:
        cursor = root.cursor 
        print(cursor.location.line, cursor.location.column, cursor.kind, level)
    if not root.child_node:
        print(f"############################## {level}") 
    for child_node in root.child_node:
        traverse_tree(child_node, level+1)

##main code###
#print(sys.argv)
if __name__ == "__main__":
    if len(sys.argv) < 4:
        print("incorrect usage. call createGraph.py program inputfile outputfilename")
        quit()

    input_code_name = sys.argv[1]
    input_file_name = sys.argv[2] 
    output_file_name = sys.argv[3]



    with open(input_file_name) as in_json_file:
        input_json = json.load(in_json_file)

    output_dict = {
    }

    output_dict.update(input_json)

    #file parsing set up
    index = clang.cindex.Index.create()
    translation_unit = index.parse(input_code_name, args=['-std=c99'])
    print("testing cfg")

    #FunctionDecl* func = /* ... */ ;
    #ASTContext* context = /* ... */ ;
    #Stmt* funcBody = func->getBody();
    #CFG* sourceCFG = CFG::buildCFG(func, funcBody, context, clang::CFG::BuildOptions());

    #add each variable in state struct as variable
    variables = get_state_variables(translation_unit)
    output_dict['variables'] = variables

    root = extract_if_tree(translation_unit.cursor)

    traverse_tree(root)

    # num_mode_params = extract_num_mode_params(translation_unit)
    modes, num_kw = extract_modes(root)
    print(modes, num_kw)

    mode_list, edges, guards, resets = extract_transitions(root)
    print(mode_list)
    print(edges)
    print(guards)
    print(resets)
    # #traverse the tree for all the paths
    # paths = traverse_tree(translation_unit.cursor, [],[], "", False)

    # vertices = []
    # counter = 0
    # for path in paths:
    #     vertices.append(str(counter))
    #     counter += 1

    # output_dict['vertex'] = vertices


    # #traverse outter if statements and find inner statements
    # edges = []
    # guards = []
    # resets = []

    # #add edge, transition(guards) and resets
    output_dict['edge'] = edges
    output_dict['guards'] = guards
    output_dict['resets'] = resets
    output_dict['vertex'] = mode_list

    output_json = json.dumps(output_dict, indent=4)
    #print(output_json)
    outfile = open(output_file_name, "w")
    outfile.write(output_json)
    outfile.close()

    # print("wrote json to " + output_file_name)










