#Accidentally uploaded old code and left computer with newest code at home!!!! 
#This is quite old! It only computes a path, but doesn't distingusih between guards and resets

from cgitb import reset
import clang.cindex
import typing
import json
import sys
from typing import List, Tuple
import re 
import itertools
import ast
#from cfg import CFG
#import clang.cfg

class logic_tree_node:
    def __init__(self, data, child = []):
        self.data = data 
        self.child = child

class if_tree_node:
    def __init__(self, cursor: clang.cindex.Cursor, children_ifs = []):
        self.child_node: List(if_tree_node) = children_ifs
        self.cursor: clang.cindex.Cursor = cursor 

class codeParser:
    def __init__(self):
        self.mode_kw_dict = {}
        self.mode_kw_order = {}
        self.mode_variable_dict = {}
    
    #get the structs
    def filter_node_list_by_node_kind(self,
        nodes: typing.Iterable[clang.cindex.Cursor],
        kinds: list
    ) -> typing.Iterable[clang.cindex.Cursor]:
        result = []
        for i in nodes:
            if i.kind in kinds:
                result.append(i)
        return result

    #exclude includes
    def filter_node_list_by_file(self,
        nodes: typing.Iterable[clang.cindex.Cursor],
        file_name: str
    ) -> typing.Iterable[clang.cindex.Cursor]:
        result = []
        for i in nodes:
            if i.location.file.name == file_name:
                result.append(i)
        return result

    #captured fields in classes
    def is_exposed_field(self, node):return node.access_specifier == clang.cindex.AccessSpecifier.PUBLIC
    def find_all_exposed_fields(
        self, 
        cursor: clang.cindex.Cursor
    ):
        result = []
        field_declarations = self.filter_node_list_by_node_kind(cursor.get_children(), [clang.cindex.CursorKind.FIELD_DECL])
        for cursor in field_declarations:
            if not self.is_exposed_field(cursor):
                continue
            if list(cursor.get_tokens())[0].spelling == "enum":
                continue
            result.append(cursor.displayname)
        return result

    def find_all_exposed_enums(
        self, 
        cursor: clang.cindex.Cursor
    ):
        result = []
        field_declarations = self.filter_node_list_by_node_kind(cursor.get_children(), [clang.cindex.CursorKind.FIELD_DECL])
        for cursor in field_declarations:
            if not self.is_exposed_field(cursor):
                continue
            if list(cursor.get_tokens())[0].spelling != "enum":
                continue
            result.append(cursor.displayname)
        return result

    def populate_field_list_recursively(self, class_name: str,class_field_map,class_inheritance_map):
        field_list = class_field_map.get(class_name)
        if field_list is None:
            return []
        baseClasses = class_inheritance_map[class_name]
        for i in baseClasses:
            field_list = self.populate_field_list_recursively(i.spelling) + field_list
        return field_list
    #rtti_map = {}

    def get_discrete_state_variables(self, translation_unit):
        rtti_map = {}
        source_nodes = self.filter_node_list_by_file(translation_unit.cursor.get_children(), translation_unit.spelling)
        state_classes = self.filter_node_list_by_node_kind(source_nodes, [clang.cindex.CursorKind.STRUCT_DECL])[0]
        mode_variable_dict = {} # Dict{mode_variable_name:mode_variable_type}
        for elem_cursor in state_classes.get_children():
            if elem_cursor.kind == clang.cindex.CursorKind.FIELD_DECL:
                if self.is_exposed_field(elem_cursor):
                    if self.get_variable_type(elem_cursor) == "enum":
                        kw = list(elem_cursor.get_tokens())[1].spelling
                        variable_name = list(elem_cursor.get_tokens())[2].spelling
                        mode_variable_dict[variable_name] = kw
        self.mode_variable_dict = mode_variable_dict
        return mode_variable_dict        

    def get_continuous_state_variables(self, translation_unit):
        rtti_map = {}
        source_nodes = self.filter_node_list_by_file(translation_unit.cursor.get_children(), translation_unit.spelling)
        all_classes = self.filter_node_list_by_node_kind(source_nodes, [clang.cindex.CursorKind.STRUCT_DECL])
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
            fields = self.find_all_exposed_fields(i)
            class_field_map[i.spelling] = fields
        #print("foo")
        for class_name, class_list in class_inheritance_map.items():
            rtti_map[class_name] = self.populate_field_list_recursively(class_name,class_field_map,class_inheritance_map)
        for class_name, field_list in rtti_map.items():
            rendered_fields = []
            for f in field_list:
                rendered_fields.append(f)
        return rendered_fields

    #given the if statement as node, visit each of the else if statements
    def visit_elif(self, node):
        print("visiting inner elseif statement")
        for i in node.get_children():
            #print(i.kind)
            #print(i.spelling)
            if i.kind == clang.cindex.CursorKind.IF_STMT or i.kind == clang.cindex.CursorKind.COMPOUND_STMT:
                print("found el-if of compound")
                print(i.spelling)
                self.visit_elif(i)
        print("finished elseif statement")    
    #if (node.hasElseStorage()):
            #print("foo")

    def get_cursor_id(self, cursor, cursor_list = []):
        if cursor is None:
            return None

        # FIXME: This is really slow. It would be nice if the index API exposed
        # something that let us hash cursors.
        for i,c in enumerate(cursor_list):
            if cursor == c:
                return i
        cursor_list.append(cursor)
        return len(cursor_list) - 1

    def get_info(self, node, depth=0):
        children = [self.get_info(c, depth+1)
                        for c in node.get_children()]
        return { 'id' : self.get_cursor_id(node),
                'kind' : node.kind,
                'usr' : node.get_usr(),
                'spelling' : node.spelling,
                'location' : node.location,
                'extent.start' : node.extent.start,
                'extent.end' : node.extent.end,
                'is_definition' : node.is_definition(),
                'definition id' : self.get_cursor_id(node.get_definition())}
                #'children' : children }

    def code_from_cursor(self, cursor):
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

    def code_from_cursor_raw(self, cursor):
        code = ""
        for tok in cursor.get_tokens():
            code += tok.spelling
        return code

    def find_immediate_if(self, cursor:clang.cindex.Cursor) -> List[if_tree_node]:
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
                res += self.find_immediate_if(child_cursor)
        return res

    # Purge the complicated cursor tree to ones that only have ifs
    # Input: cursor, the root cursor that we want to explore
    #        root, the root of the if tree
    # Return: List(if_tree_node), the list of immediate children if statement
    def extract_if_tree(self, cursor:clang.cindex.Cursor) -> if_tree_node:
        node_list = []
        root = if_tree_node(None)
        root.child_node = self.find_immediate_if(cursor)
        for node in root.child_node:
            node_list.append(node)
        while node_list:
            child_node = node_list.pop(0)
            child_cursor = child_node.cursor
            child_node.child_node = self.find_immediate_if(child_cursor)
            for node in child_node.child_node:
                node_list.append(node)
        return root

    def extract_modes(self, root: if_tree_node) -> Tuple[List[str], int]:
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

    def extract_transitions(self, root: if_tree_node):
        mode_list, num_kw = self.extract_modes(root)
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
                    guard_str = self.code_from_cursor_raw(edge_guard_cursor)
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
                        edge_resets += (self.code_from_cursor_raw(reset_cursor)+";")
                    if edge_resets != "" and edge_resets[-1] == ";":
                        edge_resets=edge_resets[:-1]
                    edge.append([edge_src_idx, edge_dest_idx])
                    guards.append(edge_guard)
                    resets.append(edge_resets)
        guards = self.pretty_guards(guards)
        return mode_list, edge, guards, resets                    

    def generate_mode_str_from_kw(self, mode_str_list):
        ordered_mode_str_list = []
        for i in range(len(self.mode_kw_dict)):
            ordered_mode_str_list.append("")
        for mode_str in mode_str_list:
            for key in self.mode_kw_dict:
                if mode_str in self.mode_kw_dict[key]:
                    break
            pos = self.mode_kw_order[key]
            ordered_mode_str_list[pos] = mode_str
        res = ""
        for mode_str in ordered_mode_str_list:
            res = res + mode_str + ";"
        if res!= "" and res[-1] == ";":
            res=res[:-1]
        return res

    def extract_transitions2(self, root: if_tree_node):
        mode_list = self.extract_all_modes(self.mode_kw_dict)
        num_kw = len(self.mode_kw_dict)
        node_list = []
        edge = []
        guards = []
        resets = []

        for node in root.child_node:
            cursor = node.cursor
            condition_cursor = list(cursor.get_children())[0]
            mode_str = list(condition_cursor.get_tokens())[2].spelling
            node_list.append((node, [mode_str]))
        while node_list:
            par_node, prev_mode_str = node_list.pop(0)
            for node in par_node.child_node:
                if node.child_node:
                    cursor = node.cursor
                    condition_cursor = list(cursor.get_children())[0]
                    mode_str = list(condition_cursor.get_tokens())[2].spelling
                    node_list.append((node, prev_mode_str + [mode_str]))
                else:
                    cursor = node.cursor 

                    # Get the source of the edge
                    if len(prev_mode_str) < num_kw:
                        raise ValueError("Insufficient number of keywords to determine transition source")
                    edge_src = self.generate_mode_str_from_kw(prev_mode_str)
                    edge_src_idx = mode_list.index(edge_src)
                    curr_mode_dict = {}
                    for val in prev_mode_str:
                        for key in self.mode_kw_dict:
                            if val in self.mode_kw_dict[key]:
                                curr_mode_dict[key] = val 
                                break

                    # Get the guard for the edge
                    edge_guard_cursor = list(cursor.get_children())[0]
                    guard_str = self.code_from_cursor_raw(edge_guard_cursor)
                    edge_guard = guard_str 

                    # Get the destination of the edge
                    compound_cursor = list(cursor.get_children())[1]
                    code_cursors = list(compound_cursor.get_children())
                    dest_mode_kw = []
                    appeared_key = []
                    num_dest_kw = 0
                    for i in range(len(code_cursors)):
                        line_cursor = code_cursors[i]
                        if line_cursor.kind == clang.cindex.CursorKind.BINARY_OPERATOR:
                            val = list(line_cursor.get_tokens())[2].spelling
                            for key in self.mode_kw_dict:
                                if val in self.mode_kw_dict[key]:
                                    appeared_key.append(key)
                                    dest_mode_kw.append(val)
                                    num_dest_kw += 1
                                    break
                    all_keys = self.mode_kw_dict.keys()
                    missing_key = all_keys - appeared_key
                    for key in missing_key:
                        dest_mode_kw.append(curr_mode_dict[key])
                    edge_dest = self.generate_mode_str_from_kw(dest_mode_kw)
                    edge_dest_idx = mode_list.index(edge_dest)
                    # for i in range(num_kw):
                    #     guard_cursor = code_cursors[i]
                    #     dest_mode_kw += [list(guard_cursor.get_tokens())[2].spelling]
                    # edge_dest = dest_mode_kw[0]
                    # for i in range(1,len(dest_mode_kw)):
                    #     edge_dest = edge_dest+";"+dest_mode_kw[i]
                    # if edge_dest not in mode_list:
                    #     mode_list.append(edge_dest)
                    # edge_dest_idx = mode_list.index(edge_dest)

                    # Get the resets for the edge
                    edge_resets = ""
                    for i in range(num_dest_kw, len(code_cursors)):
                        reset_cursor = code_cursors[i]
                        edge_resets += (self.code_from_cursor_raw(reset_cursor)+";")
                    if edge_resets != "" and edge_resets[-1] == ";":
                        edge_resets=edge_resets[:-1]
                    edge.append([edge_src_idx, edge_dest_idx])
                    guards.append(edge_guard)
                    resets.append(edge_resets)
        guards = self.pretty_guards(guards)
        return mode_list, edge, guards, resets                   

    def extract_transitions3(self, root: if_tree_node):
        mode_list = self.extract_all_modes(self.mode_kw_dict)
        num_kw = len(self.mode_kw_dict)
        node_list = []
        edge = []
        guards = []
        resets = []

        for node in root.child_node:
            cursor = node.cursor
            condition_cursor = list(cursor.get_children())[0]
            mode_str = list(condition_cursor.get_tokens())[2].spelling
            node_list.append((node, [mode_str]))
        while node_list:
            par_node, prev_mode_str = node_list.pop(0)
            for node in par_node.child_node:
                if node.child_node:
                    cursor = node.cursor
                    if cursor.kind == clang.cindex.CursorKind.IF_STMT:
                        condition_cursor = list(cursor.get_children())[0]
                        mode_str = list(condition_cursor.get_tokens())[2].spelling
                        node_list.append((node, prev_mode_str + [mode_str]))
                else:
                    cursor = node.cursor 

                    # Get the source of the edge
                    if len(prev_mode_str) < num_kw:
                        raise ValueError("Insufficient number of keywords to determine transition source")
                    edge_src = self.generate_mode_str_from_kw(prev_mode_str)
                    edge_src_idx = mode_list.index(edge_src)
                    curr_mode_dict = {}
                    for val in prev_mode_str:
                        for key in self.mode_kw_dict:
                            if val in self.mode_kw_dict[key]:
                                curr_mode_dict[key] = val 
                                break

                    # Get the guard for the edge
                    edge_guard_cursor = list(cursor.get_children())[0]
                    guard_str = self.code_from_cursor_raw(edge_guard_cursor)
                    edge_guard = guard_str 

                    # Get the destination of the edge
                    compound_cursor = list(cursor.get_children())[1]
                    code_cursors = list(compound_cursor.get_children())
                    dest_mode_kw = []
                    appeared_key = []
                    num_dest_kw = 0
                    for i in range(len(code_cursors)):
                        line_cursor = code_cursors[i]
                        if line_cursor.kind == clang.cindex.CursorKind.BINARY_OPERATOR:
                            variable = list(line_cursor.get_tokens())[0].spelling
                            if variable in self.mode_variable_dict:
                                key = self.mode_variable_dict[variable]
                                val = list(line_cursor.get_tokens())[2].spelling
                                appeared_key.append(key)
                                dest_mode_kw.append(val)
                    all_keys = self.mode_kw_dict.keys()
                    missing_key = all_keys - appeared_key
                    for key in missing_key:
                        dest_mode_kw.append(curr_mode_dict[key])
                    edge_dest = self.generate_mode_str_from_kw(dest_mode_kw)
                    edge_dest_idx = mode_list.index(edge_dest)
                    # for i in range(num_kw):
                    #     guard_cursor = code_cursors[i]
                    #     dest_mode_kw += [list(guard_cursor.get_tokens())[2].spelling]
                    # edge_dest = dest_mode_kw[0]
                    # for i in range(1,len(dest_mode_kw)):
                    #     edge_dest = edge_dest+";"+dest_mode_kw[i]
                    # if edge_dest not in mode_list:
                    #     mode_list.append(edge_dest)
                    # edge_dest_idx = mode_list.index(edge_dest)

                    # Get the resets for the edge
                    edge_resets = ""
                    for i in range(0, len(code_cursors)):
                        reset_cursor = code_cursors[i]
                        if reset_cursor.kind == clang.cindex.CursorKind.BINARY_OPERATOR:
                            variable = list(reset_cursor.get_tokens())[0].spelling
                            if variable not in self.mode_variable_dict:
                                edge_resets += (self.code_from_cursor_raw(reset_cursor)+";")
                    if edge_resets != "" and edge_resets[-1] == ";":
                        edge_resets=edge_resets[:-1]
                    edge.append([edge_src_idx, edge_dest_idx])
                    guards.append(edge_guard)
                    resets.append(edge_resets)
        guards = self.pretty_guards(guards)
        return mode_list, edge, guards, resets                   

    #assumption, word mode is not in states
    def pretty_vertices(self, vertices):
        output = []
        for vertex_code in vertices:
            parts = vertex_code.split("==")
            nonwhitespace = parts[-1].split()
            if "mode" not in nonwhitespace:
                output.append(nonwhitespace[0].strip(')'))
        return output
    
    def get_next_state(self, code):
        line = code[-2]
        parts = line.split("=")
        state = parts[-1].strip(';')
        return state.strip()

    # Split a single logic string into a list of strings 
    # with atomic logic expression, "(", ")", "&&", "||"
    def logic_string_split(self, logic_string):
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

    def generate_logic_tree(self, logic_string):
        # Construct expression tree from inorder expression
        # https://www.geeksforgeeks.org/program-to-convert-infix-notation-to-expression-tree/
        logic_string = "(" + logic_string + ")"
        s = self.logic_string_split(logic_string)
        # print(s)
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

    def generate_guard_string(self, root: logic_tree_node)->str:
        if root.data!="&&" and root.data!="||":
            return root.data
        else:
            data1 = self.generate_guard_string(root.child[0])
            data2 = self.generate_guard_string(root.child[1])
            if root.data == "&&":
                return f"And({data1},{data2})"
            elif root.data == "||":
                return f"Or({data1},{data2})"
            
    #TODO: consider or??
    def pretty_guards(self,code):
        guard_string_list = []
        for logic_string in code:
            # split_logic_string = logic_string_split(logic_string)
            # print(split_logic_string)

            root = self.generate_logic_tree(logic_string)
            guard_string = self.generate_guard_string(root)
            # print(guard_string)
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
    def pretty_resets(self, code):
        outstring = ""
        for index in range(0,len(code)):
            if index != 0 and index != len(code)-1 and index != len(code)-2:
                outstring += code[index].strip().strip('\n')
        return outstring.strip(';')

    def extract_mode_kw(self, root_cursor:clang.cindex.Cursor):
        enum_decleration = self.filter_node_list_by_node_kind(
            root_cursor.get_children(), 
            [clang.cindex.CursorKind.ENUM_DECL]
        )
        num_kw = len(enum_decleration)
        mode_kw_dict = {}
        self.mode_kw_order = {}
        for i in range(num_kw):
            enum_decleration_cursor = enum_decleration[i]
            key = list(enum_decleration_cursor.get_tokens())[1].spelling
            mode_kw_dict[key] = {}
            self.mode_kw_order[key] = i
            for j, enum_val in enumerate(enum_decleration_cursor.get_children()):
                kw = list(enum_val.get_tokens())[0].spelling
                mode_kw_dict[key][kw] = j    
        self.mode_kw_dict = mode_kw_dict
        return mode_kw_dict

    def get_variable_type(self, cursor:clang.cindex.Cursor):
        return list(cursor.get_tokens())[0].spelling

    def extract_mode_variable(self, root_cursor:clang.cindex.Cursor):
        function_decl_cursor = self.filter_node_list_by_node_kind(root_cursor.get_children(), [clang.cindex.CursorKind.FUNCTION_DECL])[0]
        function_body_cursor = self.filter_node_list_by_node_kind(function_decl_cursor.get_children(), [clang.cindex.CursorKind.COMPOUND_STMT])[0]
        variable_decleration_cursors = self.filter_node_list_by_node_kind(function_body_cursor.get_children(), [clang.cindex.CursorKind.DECL_STMT])
        mode_variable_cursors = []
        for variable_decleration_cursor in variable_decleration_cursors:
            if self.get_variable_type(variable_decleration_cursor) == "enum":
                mode_variable_cursors.append(variable_decleration_cursor)

        mode_variable_dict = {} # Dict{mode_variable_name:mode_variable_type}
        for mode_variable_cursor in mode_variable_cursors:
            kw = list(mode_variable_cursor.get_tokens())[1].spelling
            variable_name = list(mode_variable_cursor.get_tokens())[2].spelling
            mode_variable_dict[variable_name] = kw
        self.mode_variable_dict = mode_variable_dict
        return mode_variable_dict

    def extract_all_modes(self, mode_kw_dict):
        all_lists = []
        for key in mode_kw_dict:
            one_list = []
            for kw in mode_kw_dict[key]:
                one_list.append(kw)
            all_lists.append(one_list)
        res = list(itertools.product(*all_lists))
        tmp = []
        for mode_tuple in res:
            mode_str = ""
            for kw in mode_tuple:
                mode_str = mode_str + kw + ";"
            if mode_str!="" and mode_str[-1]==";":
                mode_str = mode_str[:-1]
            tmp.append(mode_str)
        return tmp

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
    code_parser = codeParser()
    variables = code_parser.get_continuous_state_variables(translation_unit)
    output_dict['variables'] = variables

    mode_kw_dict = code_parser.extract_mode_kw(translation_unit.cursor)
    print(mode_kw_dict)
    mode_variable_dict = code_parser.get_discrete_state_variables(translation_unit)
    print(mode_variable_dict)

    root = code_parser.extract_if_tree(translation_unit.cursor)

    # traverse_tree(root)

    # num_mode_params = extract_num_mode_params(translation_unit)
    # modes, num_kw = code_parser.extract_used_modes(root)
    # print(modes, num_kw)
    modes = code_parser.extract_all_modes(mode_kw_dict)
    print(modes)

    mode_list, edges, guards, resets = code_parser.extract_transitions3(root)
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










