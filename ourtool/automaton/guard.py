import re
from typing import List, Dict
from ourtool.automaton.hybrid_io_automaton import HybridIoAutomaton

class LogicTreeNode:
    def __init__(self, data, child = [], val = None, mode_guard = None):
        self.data = data 
        self.child = child
        self.val = val
        self.mode_guard = mode_guard

class Guard:
    def __init__(self, root:LogicTreeNode=None, logic_str:str=None):
        self.logic_tree_root = root
        self.logic_string = logic_str
        if self.logic_tree_root is None and logic_str is not None:
            self.construct_tree_from_str(logic_str)

    def logic_string_split(self, logic_string):
        # Input:
        #   logic_string: str, a python logic expression
        # Output:
        #   List[str], a list of string containing atomics, logic operator and brackets
        # The function take a python logic expression and split the expression into brackets, atomics and logic operators
        # logic_string = logic_string.replace(' ','')
        res = re.split('(and)',logic_string)

        tmp = []
        for sub_str in res:
            tmp += re.split('(or)',sub_str)
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
        while(" " in res):
            res.remove(" ")
        for i,sub_str in enumerate(res):
            res[i]= sub_str.strip(' ')

        # Handle spurious brackets in the splitted string
        # Get all the index of brackets pairs in the splitted string
        # Construct brackets tree
        # class BracketTreeNode:
        #     def __init__(self):
        #         self.left_idx = None 
        #         self.right_idx = None 
        #         self.child = []
        bracket_stack = []
        for i in range(len(res)):
            if res[i] == "(":
                bracket_stack.append(i)
            elif res[i] == ")":
                left_idx = bracket_stack.pop()
                sub_list = res[left_idx:i+1]
                # Check for each brackets pairs if there's any logic operators in between
                # If no, combine things in between and the brackets together, reconstruct the list
                if "and" not in sub_list and "or" not in sub_list:   
                    res[left_idx] = "".join(sub_list)
                    for j in range(left_idx+1,i+1):
                        res[j] = ""

        # For each pair of logic operator
        start_idx = 0
        end_idx = 0
        for i in range(len(res)):
            if res[i]!="(":
                start_idx = i
                break
        
        for i in range(len(res)):
            if res[i] == "and" or res[i] == "or":
                end_idx = i 
                sub_list = res[start_idx:end_idx]
                # Check if there's any dangling brackents in between. 
                # If no, combine things between logic operators
                if "(" not in sub_list and ")" not in sub_list:
                    res[start_idx] = "".join(sub_list)
                    for j in range(start_idx+1, end_idx):
                        res[j] = ""
                start_idx = end_idx + 1
        while("" in res) :
            res.remove("")
        return res 

    def construct_tree_from_str(self, logic_string:str):
        # Convert an infix expression notation to an expression tree
        # https://www.geeksforgeeks.org/program-to-convert-infix-notation-to-expression-tree/

        self.logic_string = logic_string
        logic_string = "(" + logic_string + ")"
        s = self.logic_string_split(logic_string)

        stN = []
        stC = []
        p = {}
        p["and"] = 1
        p["or"] = 1
        p[")"] = 0

        for i in range(len(s)):
            if s[i] == "(":
                stC.append(s[i])
            
            elif s[i] not in p:
                t = LogicTreeNode(s[i])
                stN.append(t)
            
            elif(p[s[i]]>0):
                while (len(stC) != 0 and stC[-1] != '(' and p[stC[-1]] >= p[s[i]]):
                                    # Get and remove the top element
                    # from the character stack
                    t = LogicTreeNode(stC[-1])
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
                    t = LogicTreeNode(stC[-1])
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
        self.logic_tree_root = t

    def generate_guard_string(self):
        return self._generate_guard_string(self.logic_tree_root)

    def _generate_guard_string(self, root: LogicTreeNode)->str:
        if root.data!="and" and root.data!="or":
            return root.data
        else:
            data1 = self._generate_guard_string(root.child[0])
            data2 = self._generate_guard_string(root.child[1])
            if root.data == "and":
                return f"And({data1},{data2})"
            elif root.data == "or":
                return f"Or({data1},{data2})"

    def execute_guard(self, aut1:HybridIoAutomaton, aut2:HybridIoAutomaton, discrete_variable_dict:Dict) -> bool:
        # This function will execute guard, and remove guard related to mode from the tree
        # We can do this recursively
        res = self._execute_guard(self.logic_tree_root, aut1, aut2, discrete_variable_dict)
        
        return res

    def _execute_guard(self, root:LogicTreeNode, aut1:HybridIoAutomaton, aut2:HybridIoAutomaton, discrete_variable_dict:Dict) -> bool:
        # If is tree leaf
        if root.child == []:
            # Check if the expression involves mode
            expr = root.data
            is_mode_guard = False
            for key in discrete_variable_dict:
                if key in expr:
                    is_mode_guard = True
                    expr = expr.replace(key, discrete_variable_dict[key])
            if is_mode_guard:
                # Execute guard, assign type and  and return result
                root.mode_guard = True
                res = eval(expr)
                root.val = res 
                return res
            # Otherwise, return True
            else: 
                root.mode_guard = False 
                root.val = True 
                return True
        # For the two children, call _execute_guard and collect result
        res1 = self._execute_guard(root.child[0],aut1,aut2,discrete_variable_dict)
        res2 = self._execute_guard(root.child[1],aut1,aut2,discrete_variable_dict)
        # Evaluate result for current node
        if root.data == "and":
            res = res1 and res2 
        elif root.data == "or":
            res = res1 or res2
        else:
            raise ValueError(f"Invalid root data {root.data}")
        
        # If the result is False, return False
        if not res:
            return False 
        # Else if any child have false result, remove that child
        else: 
            if not res1 or root.child[0].mode_guard:
                root.data = root.child[1].data
                root.val = root.child[1].val 
                root.mode_guard = root.child[1].mode_guard
                root.child = root.child[1].child
            elif not res2 or root.child[1].mode_guard:
                root.data = root.child[0].data
                root.val = root.child[0].val 
                root.mode_guard = root.child[0].mode_guard
                root.child = root.child[0].child
            return True

if __name__ == "__main__":
    tmp = Guard()
    tmp.construct_tree_from_str('(other_x-ego_x<20) and other_x-ego_x>10 and other_vehicle_lane==ego_vehicle_lane')
    print("stop")