// This file is used by Code Analysis to maintain SuppressMessage
// attributes that are applied to this project.
// Project-level suppressions either have no target or are given
// a specific target and scoped to a namespace, type, member, etc.

using System.Diagnostics.CodeAnalysis;

[assembly: SuppressMessage("Design", "CA1051:Do not declare visible instance fields", Justification = "To be fixed, there are just too many right now", Scope = "module")]
[assembly: SuppressMessage("Design", "CA1062:Validate arguments of public methods", Justification = "To be fixed, there are too many right now", Scope = "module")]
[assembly: SuppressMessage("Usage", "CA2211:Non-constant fields should not be visible", Justification = "See CA1051 above", Scope = "module")]
[assembly: SuppressMessage("Design", "CA1031:Do not catch general exception types", Justification = "To be fixed, too many to address right now", Scope = "module")]
[assembly: SuppressMessage("Globalization", "CA1307:Specify StringComparison", Justification = "To be fixed, this tool is not localized", Scope = "module")]
[assembly: SuppressMessage("Globalization", "CA1303:Do not pass literals as localized parameters", Justification = "To be fixed, this tool is not localized", Scope = "module")]
[assembly: SuppressMessage("Globalization", "CA1305:Specify IFormatProvider", Justification = "To be fixed, this tool is not localized", Scope = "module")]
[assembly: SuppressMessage("Reliability", "CA2000:Dispose objects before losing scope", Justification = "To be fixed, but too many for now", Scope = "module")]
