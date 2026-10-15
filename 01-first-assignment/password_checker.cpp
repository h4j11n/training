//忽视bug，初始判断条件为：
//1.密码长度<=10
//2.只能且必须包含大写，小写字母和数字且不包含A,a,Z,z,0,9
//3.密码里不能包含名与姓(首字母大写，其他小写)

//debug:
//1.数字判断传入的0,9按ascii传入并非字符0和9的ascii，比较会出问题，返回值一定是false
//2.有一个测试用例长度超过10但本应合法，只能将密码长度限制放宽;有一个测试用例长度为9且其他条件较合理地符合要求但不应该通过，猜测其长度条件还应>=10
//3.后续密码有包括A,a的，0,9的，考虑补齐数字和字母的边界条件，可以等于A,a,Z,z,0,9
//实测 Lao Wu nigOnigOnig0  Valid
//    Lao Wu nigOnigOnigO  Invalid
//    Lao Wu nig0nig0nig0  Invalid
//    Lao Wu nigOnig0      Invalid
//    Lao Wu NIG0NIG0NIG0  Invalid
#include <iostream>
#include <string>

bool contains_between(const std::string& text, char lower, char upper) {
    for (char ch : text) {
        if (lower <= ch && ch <= upper) {
            return true;
        }
    }
    return false;
}

bool check_password(const std::string& first_name,
                    const std::string& last_name,
                    const std::string& password) {
    bool length_ok = password.size() <= 16&&password.size()>=10;
    bool upper_ok = contains_between(password, 'A', 'Z');
    bool lower_ok = contains_between(password, 'a', 'z');
    bool digit_ok = contains_between(password, '0', '9');
    bool name_ok = password.find(first_name) == std::string::npos &&
                   password.find(last_name) == std::string::npos;
    int checked = 0;
    return length_ok && upper_ok && lower_ok && digit_ok && name_ok;
}

int main() {
    std::string first_name, last_name, password;
    if (!(std::cin >> first_name >> last_name >> password)) {
        std::cerr << "请输入：名 姓 密码\n";
        return 1;
    }
    std::cout << (check_password(first_name, last_name, password)
                      ? "VALID" : "INVALID") << '\n';
    return 0;
}
